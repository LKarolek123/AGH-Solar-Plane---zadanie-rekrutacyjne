import os
os.environ['MAVLINK20'] = '1'

import time
import math as m

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from functions import *
from errors import *
BATTERY_LIMIT_PCT = 30            # battery_remaining jest w procentach 0-100 (-1 = nieznany)
POSITION_TIMEOUT_S = 3.0          # po tylu sekundach bez pozycji sprawdzamy łącze

from conf import TAKEOFF_ALTITUDE_M


HEARTBEAT_CHECK_TIMEOUT_S = 5.0   # ile czekamy na heartbeat w jednej próbie
MAX_RECONNECT_TRIES = 3
EARTH_RADIUS_M = 6_371_000
MAX_HOME_OFFSET_M = 5
MIN_BATTERY_TO_START = 50
MIN_SATELLITES = 5
MAX_TARGET_OK_DISTANCE = 1.5 # trochę większe, żeby nie wywaliło timeerror podczas zwalniania

POSITION_ONLY_MASK = (
    mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

class Vehicle():

    """Dron z parametrami"""
    def __init__(self, conn):
        self.conn = conn
        self.position = (0.0,0.0,0.0,)
        self.battery_status = 100
        self.last_position_time = 0.0
        self.last_heartbeat_time = 0.0
        self.home = (0.0,0.0,0.0)
        self.start = (0.0,0.0,0.0)
        self.last_position_time = 0.0
        self.last_print=0.0
        self.is_armed=False
        self.ground_speed = None
        self.landed_state = None
        self.battery_status = None          # zamiast 100 - na początku nie znamy stanu
        self.last_battery_print = 0.0


    def poll(self, timeout=0.2):
        """Odbiera jedną wiadomość i potem decyduje, co z tym zrobić"""
        msg = self.conn.recv_match(
            blocking=True,
            timeout=timeout,
        )
        now = time.monotonic()
        if msg is not None:
            if msg.get_srcSystem() != self.conn.target_system:
                return
            msg_type = msg.get_type()
            if msg_type == 'GLOBAL_POSITION_INT':
                position = (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000)
                self.last_position_time = now
                self.position = position
                self.ground_speed = m.hypot(msg.vx, msg.vy) / 100    # ustawienie prędkości
    
            elif msg_type == 'BATTERY_STATUS':
                pct = msg.battery_remaining
                self.battery_status = pct
    
                if 0 <= pct <= BATTERY_LIMIT_PCT:
                    self.handle_mission('battery_status')
            
            elif msg_type == 'EXTENDED_SYS_STATE':
                    self.landed_state = msg.landed_state

        # brak pozycji przez POSITION_TIMEOUT_S -> sprawdzamy łącze
        if now - self.last_position_time > POSITION_TIMEOUT_S:
            print(f"Brak pozycji drona od {POSITION_TIMEOUT_S} s.")
            if self.try_restore_connection():
                self.last_position_time = time.monotonic()
            else:
                self.handle_mission('heartbeat_not_responding')
                return
        self.write_battery_status()
        self.write_coordinates()
    
    def write_coordinates(self):
        # wypisywanie pozycji co 1 s
        now = time.monotonic()
        if self.position is not None and now - self.last_print >= 1.0:
            lat, lon, alt = self.position
            print(f"Position: Latitude: {lat}, Longitude: {lon}, Altitude: {alt} m")
            distance = round(calculate_distance(self.position, self.start), 2)
            print(f"Odległość od startu: {distance} m")
            self.last_print = time.monotonic()


    def write_battery_status(self, interval_s=5.0):
        """Wypisuje procent baterii co interval_s sekund."""
        now = time.monotonic()
        if now - self.last_battery_print < interval_s:
            return
        if self.battery_status is None or self.battery_status < 0:
            print("Bateria: stan nieznany")
        else:
            print(f"Bateria: {self.battery_status}%")
        self.last_battery_print = now     


    def request_message_interval(self, msg_id, interval_s,):
        """Prosi autopilota o wysyłanie danej wiadomości co interval_s sekund."""
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            int(interval_s * 1_000_000),
            0, 0, 0, 0, 0,
        )


    def wait_for_ack(self, command, timeout=5):
        """Czeka na COMMAND_ACK dla danego polecenia. Przy odmowie lub braku odpowiedzi rzuca wyjątek."""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"Brak potwierdzenia polecenia {command} po {timeout} s")
            msg = self.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=remaining)
            if msg is None or msg.command != command:
                continue
            if msg.result == mavlink.MAV_RESULT_IN_PROGRESS:
                continue
            if msg.result == mavlink.MAV_RESULT_ACCEPTED:
                return True
            raise CommandError(f"Polecenie {command} odrzucone, kod {msg.result}")


    def wait_until_stable(self, max_speed=0.5, hold_s = 2.0, timeout=40): #ustalamy timeout 40, bo czujniki prędkości często pokazują 0.3 m/s w "bezruchu"
        """Czeka, aż dron przez hold_s sekund ma prędkość poziomą poniżej max_speed."""
        deadline = time.monotonic() + timeout
        stable_since = None
        while time.monotonic() < deadline:
            self.poll()
            if self.ground_speed is not None and self.ground_speed < max_speed:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= hold_s:
                    return
            else:
                print("Dron się ruszył - ponowne sprawdzenie warunków.")
                stable_since = None          # ruszył się - liczymy od nowa
        raise TimeoutError(f"Dron nie ustabilizował się nad punktem w ciągu {timeout} s")

    def land(self, timeout=90):
        """Ląduje pionowo w bieżącym miejscu i czeka na przyziemienie."""
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavlink.MAV_CMD_NAV_LAND,
            0,              # confirmation
            0,              # param1: abort alt (nieużywane w ArduCopterze)
            0,              # param2: precyzyjne lądowanie wyłączone
            0,              # param3: puste
            float('nan'),   # param4: yaw bez zmian
            0, 0,           # param5, param6: lat/lon = 0 → ląduj tu, gdzie jesteś
            0,              # param7: wysokość terenu
        )
        self.wait_for_ack(mavlink.MAV_CMD_NAV_LAND)
        print("Lądowanie rozpoczęte")

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.poll()
            if self.landed_state == mavlink.MAV_LANDED_STATE_ON_GROUND:
                print("Dron wylądował")
                return True
        raise TimeoutError(f"Dron nie wylądował w ciągu {timeout} s")

    def set_mode(self, mode_name, timeout=10):
        """Ustawia tryb lotu (np. 'GUIDED') i potwierdza zmianę w HEARTBEAT."""
        mapping = self.conn.mode_mapping()
        if mode_name not in mapping:
            raise ValueError(f"Nieznany tryb: {mode_name}")
        mode_id = mapping[mode_name]

        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # param1: używamy trybów ArduPilota
            mode_id,                                    # param2: numer trybu
            0, 0, 0, 0, 0,
        )
        self.wait_for_ack(mavlink.MAV_CMD_DO_SET_MODE, timeout)

        # ACK to nie wszystko, sprawdzamy faktyczny tryb w HEARTBEAT
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            hb = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if (hb is not None
                    and hb.get_srcSystem() == self.conn.target_system
                    and hb.get_srcComponent() == self.conn.target_component
                    and hb.custom_mode == mode_id):
                return True
        raise TimeoutError(f"Nie udało się potwierdzić zmiany trybu na {mode_name}")


    def arm(self, timeout=10):
        """Wysyła polecenie uzbrojenia drona, czeka na ACK i potwierdza stan uzbrojenia."""
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0, 0, 0, 0, 0, 0,
        )
        reasons = []
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Brak potwierdzenia polecenia uzbrojenia drona")
            msg = self.conn.recv_match(type=['COMMAND_ACK', 'STATUSTEXT'], blocking=True, timeout=remaining)
            if msg is None:
                continue
            if msg.get_type() == 'STATUSTEXT':
                if msg.text.startswith(('PreArm', 'Arm')):
                    reasons.append(msg.text)
                continue
            if msg.command != mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                continue
            if msg.result == mavlink.MAV_RESULT_IN_PROGRESS:
                continue
            if msg.result == mavlink.MAV_RESULT_ACCEPTED:
                break
            ERRORS = {
                1: "chwilowo odrzucone",
                2: "odmowa",
                3: "nieobsługiwane",
                4: "nie powiodło się",
            }
            error_message = ERRORS.get(msg.result, f"nieznany kod {msg.result}")
            details = "; ".join(reasons)
            raise ArmingError(f"Uzbrojenie odrzucone ({error_message}). {details}")

        # teraz należy potwierdzić uzbrojenie w HEARTBEAT
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if self.conn.motors_armed():
                return True
        raise TimeoutError("Nie udało się potwierdzić uzbrojenia")


    # def disarm_after_landing(self, auto_wait_s=10, timeout=10):
    #     """Rozbraja drona, ale tylko gdy stoi na ziemi"""
    #     #Ta funkcja nie jest nam potrzebna, bo silnik po ustawieniu mode 'LAND' automatycznie się rozbraja

    def check_disarm_confirmation(self, timeout=10):
        """Sprawdza, czy dron został automatycznie rozbrojony w ciągu 10 sekund od lądowania"""
        deadline = timeout + time.monotonic()
        while time.monotonic() < deadline:
            self.poll()                    
            if not self.conn.motors_armed():
                print("Dron rozbrojony automatycznie po lądowaniu")
                return True


    def takeoff(self, timeout=30):
        """Wysyła polecenie startu i czeka, aż dron osiągnie TAKEOFF_ALTITUDE_M nad Home."""
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavlink.MAV_CMD_NAV_TAKEOFF,
            0,                  # confirmation
            0, 0, 0, 0, 0, 0,   # param1–param6 nieużywane w ArduCopterze
            TAKEOFF_ALTITUDE_M,      # param7: wysokość względem Home [m]
        )
        self.wait_for_ack(mavlink.MAV_CMD_NAV_TAKEOFF)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg is None:
                continue
            alt = msg.relative_alt / 1000       # mm → m, względem Home
            if alt >= TAKEOFF_ALTITUDE_M * 0.95:     # tolerujemy 5% poniżej celu
                print(f"Osiągnięto wysokość {alt:.1f} m")
                return True
        raise TimeoutError(f"Dron nie wzbił się na wysokość {TAKEOFF_ALTITUDE_M} m po {timeout} s")

    def get_heading(self):
        """zwraca kierunek drona - trzeba ustalić względem czego on jest liczony"""


    def goto_position(self, target):
        """Przemieszcza drona do punktu o danych współrzędnych"""
        lat_int = round(target[0] * 1e7)
        lon_int = round(target[1] * 1e7)
        alt = target[2]
        
        self.conn.mav.set_position_target_global_int_send(
            0, #time_boot_ms,          # może być 0
            self.conn.target_system, self.conn.target_component, #target_system, target_component
            mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,      # układ współrzędnych
            POSITION_ONLY_MASK,             # które pola ma brać pod uwagę
            lat_int,                                    # lat_int
            lon_int,                                    # lon_int
            alt,                                    # alt [m]
            0, 0, 0,                                # vx, vy, vz
            0, 0, 0,                                # afx, afy, afz
            0, 0,                                   # yaw, yaw_rate
        )


    def wait_until_reached(self, target, tolerance_m=MAX_TARGET_OK_DISTANCE, timeout=180):
        """Pilnuje łączności i baterii w drodze do targetu"""
        self.last_position_time = time.monotonic()
        self.last_print = 0.0
        deadline = timeout+time.monotonic()
        remaining = deadline
        while calculate_distance(self.position, target) > tolerance_m and remaining>0:
            self.poll()
            remaining = deadline - time.monotonic()
        if calculate_distance(self.position, target) <=tolerance_m:
            return
        if remaining <=0:
            raise TimeoutError(f"Nie udało się dotrzeć do celu po {timeout}s.")


    def return_home(self, alt_m, tolerance_m=1.0, timeout=120):
        target = (self.home[0], self.home[1], alt_m)
        self.goto_position(target)                          # lot nad Home
        self.wait_until_reached(target, tolerance_m, timeout)  # czekanie na dotarcie
        self.wait_until_stable()        

        
    def ask_operator(self, question):
        while True:
            answer = input(f"{question} (TAK/NIE): ").strip().upper()
            if answer == "TAK":
                return True
            elif answer == "NIE":
                return False
            print("Napisz TAK lub NIE.")

    def start_flight(self):
        """Tryb GUIDED → uzbrojenie → start. Wywoływać dopiero po prepare_for_start()."""
        self.set_mode('GUIDED')
        self.arm()
        try:
            self.takeoff()
        except Exception:
            # dron jest uzbrojony, ale jeszcze na ziemi, więc rozbrojenie jest bezpieczne
            self.conn.arducopter_disarm()
            raise


    def wait_for_position(self, timeout=10):
        """Zwraca (lat [°], lon [°], relative_alt z pierwszego GLOBAL_POSITION_INT"""
        msg = self.conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
        if msg is None:
            raise TimeoutError('Brak pozycji drona przy starcie')
        return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000


    def check_connection(self, timeout=HEARTBEAT_CHECK_TIMEOUT_S):
        """True, jeśli w ciągu `timeout` s przyszedł heartbeat od NASZEGO autopilota."""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            msg = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=remaining)
            if msg is None:
                return False
            if (msg.get_srcSystem() == self.conn.target_system
                    and msg.get_srcComponent() == self.conn.target_component):
                return True
            # heartbeat od kogoś innego (np. stacji naziemnej) - czekamy dalej


    def try_restore_connection(self):
        """Próbuje n razy złapać heartbeat"""
        for attempt in range(1, MAX_RECONNECT_TRIES + 1):
            if self.check_connection():
                print("Heartbeat OK - łączność jest")
                return True
            print(f"Próba {attempt}/{MAX_RECONNECT_TRIES}: brak heartbeat")
        return False


    def fallback_return_plan(self):
        pass


    def handle_mission(self, reason):
        """Na podstawie błędów łączności / niskiego poziomu baterii decyduje, czy przerwać misję"""
        match reason:
            case 'battery_status':
                # bateria poniżej BATTERY_LIMIT_PCT
                self.fallback_return_plan()
            case 'heartbeat_not_responding':
                self.fallback_return_plan()
            case 'sys_error':
                self.fallback_return_plan()
            case 'connection_error':
                self.fallback_return_plan()


    def check_GPS(self, timeout=30):
        """Czeka, aż GPS ma fix 3D i wystarczająco satelitów. Inaczej rzuca wyjątek."""
        deadline = time.monotonic() + timeout
        last_problem = "brak wiadomości GPS_RAW_INT"
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"Brak gotowości GPS po {timeout} s: {last_problem}")
            msg = self.conn.recv_match(type='GPS_RAW_INT', blocking=True, timeout=remaining)
            if msg is None:
                continue
            if msg.fix_type < 3:      # potrzebny fix 3D (lub lepszy: DGPS, RTK)
                last_problem = f"fix_type = {msg.fix_type} < 3"
                continue
            if msg.satellites_visible < MIN_SATELLITES:
                last_problem = f"za mało satelitów: {msg.satellites_visible} < {MIN_SATELLITES}"
                continue
            # warto także dodać eph <= 140
            return True


    def check_EKF(self, timeout=90):
        """Czeka, aż EKF ma pewną pozycję poziomą, wysokość i prędkość."""
        EKF_REQUIRED = (
            mavlink.EKF_POS_HORIZ_ABS | mavlink.EKF_POS_VERT_ABS | mavlink.EKF_VELOCITY_HORIZ
        )
        EKF_BAD = mavlink.EKF_CONST_POS_MODE | getattr(mavlink, 'EKF_UNINITIALIZED', 0)

        last_problem = "brak wiadomości EKF_STATUS_REPORT"
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"Nie udało się potwierdzić EKF po {timeout} s: {last_problem}")
            msg = self.conn.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=remaining)
            if msg is None:
                continue
            if (msg.flags & EKF_REQUIRED) != EKF_REQUIRED:
                last_problem = f"brak wymaganych flag (flags = {bin(msg.flags)})"
            elif msg.flags & EKF_BAD:
                last_problem = f"EKF w złym stanie (flags = {bin(msg.flags)})"
            else:
                return True


    def check_the_battery(self, timeout=10):
        msg = self.conn.recv_match(type='BATTERY_STATUS', blocking=True, timeout=timeout)
        if msg is None:
            raise TimeoutError("Nie udało się odczytać stanu baterii")
        return msg.battery_remaining


    def read_home_position(self, timeout=10):
        """Prosi autopilota o HOME_POSITION i zwraca (lat [°], lon [°], alt [m n.p.m.])."""
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavlink.MAVLINK_MSG_ID_HOME_POSITION,
            0, 0, 0, 0, 0, 0,
        )
        msg = self.conn.recv_match(type='HOME_POSITION', blocking=True, timeout=timeout)
        if msg is None:
            raise TimeoutError("Nie udało się odczytać współrzędnych Home")
        return msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1000


    def prepare_for_start(self):
        """Same sprawdzenia przed startem - nic tu nie uzbraja drona."""
        if self.conn.motors_armed():
            raise ArmingError("Dron jest już uzbrojony")

        # 1: łączność
        if not self.check_connection():
            raise ConnectionError("Brak łączności z autopilotem")

        # 2: GPS + EKF (każda funkcja rzuca wyjątek, jeśli nie są gotowe)
        self.check_GPS()
        self.check_EKF()
        print("Potwierdzono GPS i EKF")
        # 3: punkt Home i aktualna pozycja drona
        home = self.read_home_position()
        start = self.wait_for_position()
        if horizontal_distance(start, home) > MAX_HOME_OFFSET_M:
            if not self.ask_operator("Pozycja drona różni się od Home. Kontynuować?"):
                raise DifferentLandingLocation("Pozycja drona różni się od punktu Home")

        # 4: poziom baterii powyżej progu
        battery_percentage = self.check_the_battery()
        print(f"Poziom baterii: {battery_percentage}%")
        if battery_percentage == -1:
            raise ValueError("Nie udało się poprawnie odczytać stanu baterii")
        if battery_percentage < MIN_BATTERY_TO_START:
            raise LowBatteryWarning(
                f"Bateria {battery_percentage}%, minimum do startu to {MIN_BATTERY_TO_START}%"
            )
        if self.ask_operator("Przygotowanie do startu zakończone sukcesem! Czy rozpocząć lot?"):
            return start, home, battery_percentage
        else:
            raise KeyError("Brak zgody na start. Wstrzymano procedurę startu.")


    def start_mission(self,start):
        """Główna funkcja misji naszego drona. Przyjmuje start jako krotkę współrzędnych punktu startowego."""
        target = destination_point(start[0], start[1], 45, 100)
        #target = (lat, lon, rel_alt)
        self.goto_position(target)
        