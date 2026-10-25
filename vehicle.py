"""Warstwa komunikacji z autopilotem (ArduCopter przez MAVLink).

Vehicle wykrywa problemy i ZGŁASZA je wyjątkami. Decyzje (powrót, przerwanie misji)
podejmuje mission.py.
"""
import os
os.environ['MAVLINK20'] = '1'

import logging
import math as m
import time

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from conf import (
    TAKEOFF_ALTITUDE_M, POSITION_TIMEOUT_S, HEARTBEAT_CHECK_TIMEOUT_S, MAX_RECONNECT_TRIES,
    MAX_HOME_OFFSET_M, MIN_BATTERY_TO_START, MIN_SATELLITES, BATTERY_LIMIT_PCT,
    BATTERY_CAPACITY_MAH_FALLBACK, TARGET_REACHED_TOLERANCE_M, SETPOINT_RATE_HZ,
    POSITION_LOG_INTERVAL_S, BATTERY_LOG_INTERVAL_S,
)
from errors import (
    ArmingError, CommandError, DifferentLandingLocation, LowBatteryWarning, OperatorAbort,
    LinkLostError, LowBatteryInFlight, PilotOverrideError,
)
from functions import calculate_distance, horizontal_distance

log = logging.getLogger("drone.vehicle")

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


class Vehicle:
    """Dron: połączenie z autopilotem i jego aktualny stan."""

    def __init__(self, conn):
        self.conn = conn

        # stan z telemetrii (wysokości zawsze względem Home)
        self.position = None              # (lat, lon, rel_alt)
        self.ground_speed = None          # m/s
        self.landed_state = None          # MAV_LANDED_STATE_*
        self.battery_status = None        # % (-1 = nieznany)
        self.current_a = None             # bieżący pobór prądu [A]
        self.consumed_mah = None          # zużycie od startu autopilota [mAh]
        self.custom_mode = None           # numer trybu z HEARTBEAT
        self.battery_capacity_mah = BATTERY_CAPACITY_MAH_FALLBACK

        self.home = None                  # (lat, lon, 0.0) - Home jest zerem wysokości
        self.home_amsl_m = None
        self.start = None

        # czasy
        self.last_position_time = time.monotonic()
        self.last_heartbeat_time = time.monotonic()
        self.last_print = 0.0
        self.last_battery_print = 0.0

        # strażnicy - włączane przez misję dopiero w powietrzu
        self.link_watch = False           # pilnuj utraty łączności
        self.battery_floor_watch = False  # pilnuj twardego progu baterii
        self.expected_mode = None         # np. 'GUIDED'; inny tryb = ktoś przejął drona
        self.monitors = []                # funkcje wołane po każdym poll(), np. strażnik energii

        # nagrywanie prądu (do liczenia średniej)
        self._current_samples = None

    # =========================================================================
    # Odbieranie telemetrii
    # =========================================================================
    def _recv(self, **kwargs):
        """recv_match odporne na zerwane połączenie: przy błędzie gniazda zwraca None.

        Przy autoreconnect pymavlink sam próbuje się połączyć ponownie, a gdy się nie uda,
        rzuca wyjątek z recv_match. Traktujemy to jak brak wiadomości - utratę łączności
        wykrywa wtedy poll() (brak pozycji) albo zwykły timeout danej operacji.
        """
        try:
            return self.conn.recv_match(**kwargs)
        except (OSError, AttributeError) as e:
            if isinstance(e, AttributeError) and getattr(self.conn, 'port', True) is not None:
                raise                        # prawdziwy błąd w kodzie, nie zerwane gniazdo
            log.debug("Błąd odbioru (zerwane połączenie?): %s", e)
            time.sleep(min(kwargs.get('timeout') or 0.5, 0.5))
            return None

    def poll(self, timeout=0.2):
        """Odbiera jedną wiadomość, aktualizuje stan i sprawdza bezpieczeństwo."""
        msg = self._recv(blocking=True, timeout=max(timeout, 0.01))
        now = time.monotonic()

        if msg is not None and msg.get_srcSystem() == self.conn.target_system:
            self._handle_message(msg, now)

        # --- łączność
        if self.link_watch and now - self.last_position_time > POSITION_TIMEOUT_S:
            log.warning("Brak pozycji drona od %.0f s - sprawdzam łącze", POSITION_TIMEOUT_S)
            if self.try_restore_connection():
                self.last_position_time = time.monotonic()
            else:
                raise LinkLostError("Utracono łączność z autopilotem")

        # --- tryb lotu
        if self.expected_mode is not None and self.custom_mode is not None:
            expected_id = self.conn.mode_mapping().get(self.expected_mode)
            if expected_id is not None and self.custom_mode != expected_id:
                raise PilotOverrideError(
                    f"Tryb zmieniony z zewnątrz na {self.mode_name(self.custom_mode)} "
                    f"(oczekiwano {self.expected_mode})"
                )

        # --- twardy próg baterii
        if (self.battery_floor_watch and self.battery_status is not None
                and 0 <= self.battery_status <= BATTERY_LIMIT_PCT):
            raise LowBatteryInFlight(f"Bateria {self.battery_status}% <= {BATTERY_LIMIT_PCT}%")

        for monitor in self.monitors:
            monitor(self)

        self.write_battery_status()
        self.write_coordinates()

    def _handle_message(self, msg, now):
        msg_type = msg.get_type()
        if msg_type == 'GLOBAL_POSITION_INT':
            self.position = (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000)
            self.ground_speed = m.hypot(msg.vx, msg.vy) / 100
            self.last_position_time = now

        elif msg_type == 'BATTERY_STATUS':
            self.battery_status = msg.battery_remaining
            if msg.current_battery >= 0:                 # cA, -1 = brak pomiaru
                self.current_a = msg.current_battery / 100
                if self._current_samples is not None:
                    self._current_samples.append(self.current_a)
            if msg.current_consumed >= 0:                # mAh, -1 = brak pomiaru
                self.consumed_mah = msg.current_consumed

        elif msg_type == 'EXTENDED_SYS_STATE':
            self.landed_state = msg.landed_state

        elif msg_type == 'HEARTBEAT':
            if msg.get_srcComponent() == self.conn.target_component:
                self.last_heartbeat_time = now
                self.custom_mode = msg.custom_mode

        elif msg_type == 'STATUSTEXT':
            if msg.severity <= mavlink.MAV_SEVERITY_WARNING:
                log.warning("[AUTOPILOT] %s", msg.text)
            else:
                log.info("[AUTOPILOT] %s", msg.text)

    def mode_name(self, mode_id):
        for name, value in self.conn.mode_mapping().items():
            if value == mode_id:
                return name
        return str(mode_id)

    def start_current_recording(self):
        self._current_samples = []

    def stop_current_recording(self):
        samples, self._current_samples = self._current_samples or [], None
        return samples

    def write_coordinates(self):
        """Wypisuje pozycję co POSITION_LOG_INTERVAL_S sekund."""
        now = time.monotonic()
        if self.position is not None and now - self.last_print >= POSITION_LOG_INTERVAL_S:
            lat, lon, alt = self.position
            line = f"Pozycja: {lat:.7f}, {lon:.7f}, wys. {alt:.1f} m"
            if self.home is not None:
                line += f" | od Home: {horizontal_distance(self.position, self.home):.1f} m"
            log.info(line)
            self.last_print = now

    def write_battery_status(self, interval_s=BATTERY_LOG_INTERVAL_S):
        """Wypisuje stan baterii co interval_s sekund."""
        now = time.monotonic()
        if now - self.last_battery_print < interval_s:
            return
        if self.battery_status is None or self.battery_status < 0:
            log.info("Bateria: stan nieznany")
        else:
            current = f", {self.current_a:.1f} A" if self.current_a is not None else ""
            log.info("Bateria: %d%%%s", self.battery_status, current)
        self.last_battery_print = now

    # =========================================================================
    # Polecenia
    # =========================================================================
    def request_message_interval(self, msg_id, interval_s):
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
            msg = self._recv(type='COMMAND_ACK', blocking=True, timeout=remaining)
            if msg is None or msg.command != command:
                continue
            if msg.result == mavlink.MAV_RESULT_IN_PROGRESS:
                continue
            if msg.result == mavlink.MAV_RESULT_ACCEPTED:
                return True
            raise CommandError(f"Polecenie {command} odrzucone, kod {msg.result}")

    def read_param(self, name, timeout=5):
        """Odczytuje parametr autopilota (np. 'BATT_CAPACITY'). Zwraca float albo None."""
        self.conn.mav.param_request_read_send(
            self.conn.target_system, self.conn.target_component, name.encode(), -1
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self._recv(type='PARAM_VALUE', blocking=True,
                                       timeout=deadline - time.monotonic())
            if msg is None:
                continue
            param_id = msg.param_id
            if isinstance(param_id, bytes):
                param_id = param_id.decode(errors='ignore')
            if param_id.rstrip('\x00') == name:
                return float(msg.param_value)
        return None

    def set_mode(self, mode_name, timeout=10):
        """Ustawia tryb lotu (np. 'GUIDED') i potwierdza zmianę w HEARTBEAT."""
        mapping = self.conn.mode_mapping()
        if mode_name not in mapping:
            raise ValueError(f"Nieznany tryb: {mode_name}")
        mode_id = mapping[mode_name]
        self.expected_mode = None        # na czas zmiany nie traktujemy innego trybu jako przejęcia

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
            hb = self._recv(type='HEARTBEAT', blocking=True, timeout=1)
            if (hb is not None
                    and hb.get_srcSystem() == self.conn.target_system
                    and hb.get_srcComponent() == self.conn.target_component
                    and hb.custom_mode == mode_id):
                self.custom_mode = mode_id
                self.expected_mode = mode_name
                log.info("Tryb lotu: %s", mode_name)
                return True
        raise TimeoutError(f"Nie udało się potwierdzić zmiany trybu na {mode_name}")

    def set_speed(self, speed_mps):
        """Ustawia prędkość przelotu (MAV_CMD_DO_CHANGE_SPEED). Zwraca True, jeśli przyjęto."""
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            1,              # param1: prędkość względem ziemi
            speed_mps,      # param2: prędkość [m/s]
            -1,             # param3: przepustnica bez zmian
            0, 0, 0, 0,
        )
        try:
            self.wait_for_ack(mavlink.MAV_CMD_DO_CHANGE_SPEED)
        except (CommandError, TimeoutError) as e:
            log.warning("Nie udało się ustawić prędkości %.1f m/s: %s", speed_mps, e)
            return False
        log.info("Prędkość przelotu: %.1f m/s", speed_mps)
        return True

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
            msg = self._recv(type=['COMMAND_ACK', 'STATUSTEXT'], blocking=True, timeout=remaining)
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
            self._recv(type='HEARTBEAT', blocking=True, timeout=1)
            if self.conn.motors_armed():
                log.info("Dron uzbrojony")
                return True
        raise TimeoutError("Nie udało się potwierdzić uzbrojenia")

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
            msg = self._recv(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg is None:
                continue
            self._handle_message(msg, time.monotonic())   # aktualizuje self.position
            alt = msg.relative_alt / 1000       # mm → m, względem Home
            if alt >= TAKEOFF_ALTITUDE_M * 0.95:     # tolerujemy 5% poniżej celu
                log.info("Osiągnięto wysokość %.1f m", alt)
                return True
        raise TimeoutError(f"Dron nie wzbił się na wysokość {TAKEOFF_ALTITUDE_M} m po {timeout} s")

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

    def goto_position(self, target):
        """Wysyła punkt docelowy (lat, lon, rel_alt). Nie czeka na dotarcie."""
        lat_int = round(target[0] * 1e7)
        lon_int = round(target[1] * 1e7)
        alt = target[2]

        self.conn.mav.set_position_target_global_int_send(
            0,                                          # time_boot_ms
            self.conn.target_system, self.conn.target_component,
            mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # wysokość względem Home
            POSITION_ONLY_MASK,                         # bierzemy pod uwagę tylko pozycję
            lat_int,
            lon_int,
            alt,                                        # [m]
            0, 0, 0,                                    # vx, vy, vz
            0, 0, 0,                                    # afx, afy, afz
            0, 0,                                       # yaw, yaw_rate
        )

    def follow_trajectory(self, point_fn, duration_s, alt_m, on_tick=None):
        """Śledzi trajektorię: co 1/SETPOINT_RATE_HZ s wysyła punkt point_fn(t) -> (lat, lon).

        duration_s może być liczbą albo funkcją bez argumentów (czas można skracać w trakcie lotu).
        on_tick(t) jest wołane w każdej iteracji; może rzucić wyjątek, żeby przerwać lot.
        Zwraca czas, przez jaki faktycznie leciano.
        """
        get_duration = duration_s if callable(duration_s) else (lambda: duration_s)
        period = 1.0 / SETPOINT_RATE_HZ
        t0 = time.monotonic()
        next_send = t0
        while True:
            now = time.monotonic()
            t = now - t0
            if t >= get_duration():
                return t
            if now >= next_send:
                lat, lon = point_fn(t)
                self.goto_position((lat, lon, alt_m))
                next_send += period
            if on_tick is not None:
                on_tick(t)
            self.poll(timeout=min(period, max(next_send - time.monotonic(), 0.01)))

    def wait_until_reached(self, target, tolerance_m=TARGET_REACHED_TOLERANCE_M, timeout=180):
        """Czeka na dotarcie do celu (odległość 3D, wysokości względem Home)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.poll()
            if self.position is not None and calculate_distance(self.position, target) <= tolerance_m:
                return True
        raise TimeoutError(f"Nie udało się dotrzeć do celu po {timeout} s")

    def wait_until_stable(self, max_speed=0.5, hold_s=2.0, timeout=40):
        """Czeka, aż dron przez hold_s sekund ma prędkość poziomą poniżej max_speed."""
        deadline = time.monotonic() + timeout
        stable_since = None
        while time.monotonic() < deadline:
            self.poll()
            if self.ground_speed is not None and self.ground_speed < max_speed:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= hold_s:
                    return True
            else:
                stable_since = None          # ruszył się - liczymy od nowa
        raise TimeoutError(f"Dron nie ustabilizował się nad punktem w ciągu {timeout} s")

    def return_home(self, alt_m, tolerance_m=1.0, timeout=180):
        """Lot nad Home na wysokości alt_m i zawiśnięcie."""
        target = (self.home[0], self.home[1], alt_m)
        log.info("Powrót nad Home")
        self.goto_position(target)
        self.wait_until_reached(target, tolerance_m, timeout)
        self.wait_until_stable()
        log.info("Dron zawisł nad Home")

    def land(self, timeout=120):
        """Ląduje pionowo w bieżącym miejscu i czeka na przyziemienie."""
        self.expected_mode = None        # NAV_LAND przełącza autopilota w tryb LAND
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
        log.info("Lądowanie rozpoczęte")

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.poll()
            if self.landed_state == mavlink.MAV_LANDED_STATE_ON_GROUND:
                log.info("Dron wylądował")
                return True
        raise TimeoutError(f"Dron nie wylądował w ciągu {timeout} s")

    def check_disarm_confirmation(self, timeout=10):
        """Sprawdza, czy dron rozbroił się automatycznie po lądowaniu (ArduCopter robi to sam)."""
        deadline = timeout + time.monotonic()
        while time.monotonic() < deadline:
            self.poll()
            if not self.conn.motors_armed():
                log.info("Dron rozbrojony automatycznie po lądowaniu")
                return True
        log.warning("Dron nie rozbroił się w ciągu %d s od lądowania", timeout)
        return False

    def emergency_return(self):
        """Próbuje bezpiecznie sprowadzić drona: najpierw RTL, a jeśli się nie uda, LAND."""
        self.monitors = []
        self.link_watch = False
        self.battery_floor_watch = False
        for mode in ('RTL', 'LAND'):
            try:
                self.set_mode(mode)
                self.expected_mode = None
                log.warning("Awaryjnie przełączono w tryb %s", mode)
                return True
            except Exception as e:
                log.error("Nie udało się przełączyć w %s: %s", mode, e)
        log.critical("Dron jest w powietrzu bez kontroli programu - przejmij go w Mission Plannerze! "
                     "(failsafe autopilota powinien zadziałać sam)")
        return False

    # =========================================================================
    # Przygotowanie przed startem
    # =========================================================================
    def ask_operator(self, question):
        while True:
            answer = input(f"{question} (TAK/NIE): ").strip().upper()
            if answer == "TAK":
                return True
            elif answer == "NIE":
                return False
            print("Napisz TAK lub NIE.")

    def wait_for_position(self, timeout=10):
        """Zwraca (lat [°], lon [°], relative_alt [m]) z pierwszego GLOBAL_POSITION_INT."""
        msg = self._recv(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
        if msg is None:
            raise TimeoutError('Brak pozycji drona przy starcie')
        self.position = (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000)
        self.last_position_time = time.monotonic()
        return self.position

    def check_connection(self, timeout=HEARTBEAT_CHECK_TIMEOUT_S):
        """True, jeśli w ciągu `timeout` s przyszedł heartbeat od NASZEGO autopilota."""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            msg = self._recv(type='HEARTBEAT', blocking=True, timeout=remaining)
            if msg is None:
                return False
            if (msg.get_srcSystem() == self.conn.target_system
                    and msg.get_srcComponent() == self.conn.target_component):
                self.last_heartbeat_time = time.monotonic()
                return True
            # heartbeat od kogoś innego (np. stacji naziemnej) - czekamy dalej

    def try_restore_connection(self):
        """Próbuje MAX_RECONNECT_TRIES razy złapać heartbeat."""
        for attempt in range(1, MAX_RECONNECT_TRIES + 1):
            if self.check_connection():
                log.info("Heartbeat OK - łączność jest")
                return True
            log.warning("Próba %d/%d: brak heartbeat", attempt, MAX_RECONNECT_TRIES)
        return False

    def check_GPS(self, timeout=30):
        """Czeka, aż GPS ma fix 3D i wystarczająco satelitów. Inaczej rzuca wyjątek."""
        deadline = time.monotonic() + timeout
        last_problem = "brak wiadomości GPS_RAW_INT"
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"Brak gotowości GPS po {timeout} s: {last_problem}")
            msg = self._recv(type='GPS_RAW_INT', blocking=True, timeout=remaining)
            if msg is None:
                continue
            if msg.fix_type < 3:      # potrzebny fix 3D (lub lepszy: DGPS, RTK)
                last_problem = f"fix_type = {msg.fix_type} < 3"
                continue
            if msg.satellites_visible < MIN_SATELLITES:
                last_problem = f"za mało satelitów: {msg.satellites_visible} < {MIN_SATELLITES}"
                continue
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
            msg = self._recv(type='EKF_STATUS_REPORT', blocking=True, timeout=remaining)
            if msg is None:
                continue
            if (msg.flags & EKF_REQUIRED) != EKF_REQUIRED:
                last_problem = f"brak wymaganych flag (flags = {bin(msg.flags)})"
            elif msg.flags & EKF_BAD:
                last_problem = f"EKF w złym stanie (flags = {bin(msg.flags)})"
            else:
                return True

    def check_the_battery(self, timeout=10):
        msg = self._recv(type='BATTERY_STATUS', blocking=True, timeout=timeout)
        if msg is None:
            raise TimeoutError("Nie udało się odczytać stanu baterii")
        self.battery_status = msg.battery_remaining
        if msg.current_consumed >= 0:
            self.consumed_mah = msg.current_consumed
        return msg.battery_remaining

    def read_home_position(self, timeout=10):
        """Prosi autopilota o HOME_POSITION i zwraca (lat, lon, 0.0).
        Wysokość Home względem Home to z definicji 0; wysokość n.p.m. zapisujemy osobno."""
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavlink.MAVLINK_MSG_ID_HOME_POSITION,
            0, 0, 0, 0, 0, 0,
        )
        msg = self._recv(type='HOME_POSITION', blocking=True, timeout=timeout)
        if msg is None:
            raise TimeoutError("Nie udało się odczytać współrzędnych Home")
        self.home_amsl_m = msg.altitude / 1000
        return msg.latitude / 1e7, msg.longitude / 1e7, 0.0

    def read_battery_capacity(self):
        """Pojemność baterii z parametru BATT_CAPACITY (mAh)."""
        value = self.read_param('BATT_CAPACITY')
        if value and value > 0:
            self.battery_capacity_mah = value
        else:
            log.warning("Nie udało się odczytać BATT_CAPACITY - przyjmuję %d mAh",
                        BATTERY_CAPACITY_MAH_FALLBACK)
            self.battery_capacity_mah = BATTERY_CAPACITY_MAH_FALLBACK
        return self.battery_capacity_mah

    def prepare_for_start(self, confirm=True):
        """Same sprawdzenia przed startem - nic tu nie uzbraja drona."""
        if self.conn.motors_armed():
            raise ArmingError("Dron jest już uzbrojony")

        # 1: łączność
        if not self.check_connection():
            raise ConnectionError("Brak łączności z autopilotem")

        # 2: GPS + EKF (każda funkcja rzuca wyjątek, jeśli nie są gotowe)
        self.check_GPS()
        self.check_EKF()
        log.info("Potwierdzono GPS i EKF")

        # 3: punkt Home i aktualna pozycja drona
        home = self.read_home_position()
        start = self.wait_for_position()
        offset = horizontal_distance(start, home)
        if offset > MAX_HOME_OFFSET_M:
            if not self.ask_operator(f"Pozycja drona różni się od Home o {offset:.1f} m. Kontynuować?"):
                raise DifferentLandingLocation("Pozycja drona różni się od punktu Home")
        self.home = home
        self.start = start

        # 4: poziom baterii powyżej progu
        battery_percentage = self.check_the_battery()
        capacity = self.read_battery_capacity()
        log.info("Poziom baterii: %d%% (pojemność %.0f mAh)", battery_percentage, capacity)
        if battery_percentage == -1:
            raise ValueError("Nie udało się poprawnie odczytać stanu baterii")
        if battery_percentage < MIN_BATTERY_TO_START:
            raise LowBatteryWarning(
                f"Bateria {battery_percentage}%, minimum do startu to {MIN_BATTERY_TO_START}%"
            )
        if confirm and not self.ask_operator("Przygotowanie do startu zakończone sukcesem! Czy rozpocząć lot?"):
            raise OperatorAbort("Brak zgody na start. Wstrzymano procedurę startu.")
        return start, home, battery_percentage
