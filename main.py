import os
os.environ['MAVLINK20'] = '1'

import time
import math as m

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from conf import MAVLINK_CONNECTION, MAVLINK_DIALECT, SOURCE_SYSTEM, HEARTBEAT_TIMEOUT_S

BATTERY_LIMIT_PCT = 30            # battery_remaining jest w procentach 0-100 (-1 = nieznany)
POSITION_TIMEOUT_S = 3.0          # po tylu sekundach bez pozycji sprawdzamy łącze
HEARTBEAT_CHECK_TIMEOUT_S = 2.0   # ile czekamy na heartbeat w jednej próbie
MAX_RECONNECT_TRIES = 3
EARTH_RADIUS_M = 6_371_000
MAX_HOME_OFFSET_M = 5
MIN_BATTERY_TO_START = 50

class PreflightWarning(Exception):
    """Wspólna klasa bazowa dla ostrzeżeń przed startem."""

class LowBatteryWarning(PreflightWarning):
    pass

class DifferentLandingLocation(PreflightWarning):
    pass

class ArmingError(Exception):
    pass



#tworzymy polaczenie na odpowiednim porcie z conf
conn = mavutil.mavlink_connection(
    MAVLINK_CONNECTION,
    dialect=MAVLINK_DIALECT,
    source_system=SOURCE_SYSTEM,   
)

heartbeat = conn.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S)
if heartbeat is None:
    raise TimeoutError('Brak heartbeat autopilota')
print(conn.target_system, conn.target_component)

def ask_operator(question):
    while True:
        answer = input(f"{question} (TAK/NIE)").strip().upper()
        if answer == "TAK":
            return True
        elif answer == "NIE":
            return False
        print("Napisz TAK lub NIE.")

def request_message_interval(msg_id, interval_s):
    """Prosi autopilota o wysyłanie danej wiadomości co interval_s sekund."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        int(interval_s * 1_000_000),
        0, 0, 0, 0, 0,
    )

def arm(timeout=10):
    """Funkcja wysyła polecenie uzbrojenia drona, czeka na ACK i potwierdza stan uzbrojenia"""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,0,0,0,0,0,
    )
    reasons = []
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("Brak potwierdzenia polecenia uzbrojenia drona")
        msg = conn.recv_match(type=['COMMAND_ACK', 'STATUSTEXT'], blocking=True, timeout=remaining)
        if msg is None:
            continue
        success = False
        if msg.get_type() == 'STATUSTEXT':
            if msg.text.startswith(('PreArm', 'Arm')):
                reasons.append(msg.text)
            continue
        if msg.command != mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            continue
        if msg.result == mavlink.MAV_RESULT_IN_PROGRESS:
            continue
        if msg.result == mavlink.MAV_RESULT_ACCEPTED:
            #success
            break
        ERRORS = {
            1: "chwilowo odrzucone",
            2: "odmowa",
            3: "nieobsługiwane",
            4: "nie powiodło się"
        }
        error_message = ERRORS.get(msg.result, f"nieznany kod {msg.result}") 
        details=";".join(reasons)
        raise ArmingError(f"Uzbrojenie odrzucone: ({error_message})")
    
    #teraz należy potwierdzić uzbrojenie w Heartbeat
    deadline = timeout + time.monotonic()
    while True:
        if deadline - time.monotonic() <= 0:
            raise TimeoutError("Nie udało się potwierdzić uzbrojenia")
        conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        is_armed = conn.motors_armed()
        if is_armed:
            #sukces
            return True

        
def wait_for_position(timeout=10):
    """Zwraca (lat [°], lon [°], alt [m n.p.m.]) z pierwszego GLOBAL_POSITION_INT"""
    msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
    if msg is None:
        raise TimeoutError('Brak pozycji drona przy starcie')
    return msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000


def calculate_distance(start, current):
    """Odległość w metrach między dwoma punktami (lat, lon, alt).
    Stopnie zamieniamy na metry (przybliżenie równoprostokątne -
    wystarczająco dokładne dla odległości do kilkudziesięciu km).
    """
    lat1, lon1, alt1 = start
    lat2, lon2, alt2 = current
    mean_lat = m.radians((lat1 + lat2) / 2)
    dx = m.radians(lon2 - lon1) * m.cos(mean_lat) * EARTH_RADIUS_M
    dy = m.radians(lat2 - lat1) * EARTH_RADIUS_M
    dz = alt2 - alt1
    return round(m.sqrt(dx * dx + dy * dy + dz * dz), 3)

def check_connection(timeout =HEARTBEAT_CHECK_TIMEOUT_S):
    """True, jeśli w ciągu `timeout` s przyszedł heartbeat od NASZEGO autopilota."""
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=remaining)
        if msg is None:
            return False
        if (msg.get_srcSystem() == conn.target_system
                and msg.get_srcComponent() == conn.target_component):
            return True
        # heartbeat od kogoś innego (np. stacji naziemnej) - czekamy dalej 

def try_restore_connection():
    """Probuje n razy zlapac heartbeat"""    
    for attempt in range(1, MAX_RECONNECT_TRIES+1):
        if check_connection():
            print("Heartbeat OK - łączność jest")
            return True
        print(f"Próba {attempt}/{MAX_RECONNECT_TRIES}: brak heartbeat")
    return False

def fallback_return_plan():
    pass

def handle_mission(reason):
    """Funkcja na podstawie błędów w łącznoności / niskiego poziomu baterii, podejmuje decyzję, czy przerwać misję"""
    match reason:
        case 'battery_status':
            #battery status poniżej 56%
            fallback_return_plan()
        case 'heartbeat_not_responding':
            pass
        case 'sys_error':
            pass
        case 'connection_error':
            pass

def check_GPS(timeout=30):
    """Czeka, aż GPS ma fix 3D i wystarczająco satelitów. Inaczej rzuca wyjątek."""
    deadline = timeout + time.monotonic()
    last_problem= "brak wiadomości GPS_RAW_INT"
    while True:
        if time.monotonic() >= deadline:
            raise TimeoutError(f"Brak gotowości GPS: {last_problem} po {timeout} s")
        msg = conn.recv_match(type = 'GPS_RAW_INT', blocking = True, timeout=timeout )
        if msg.fix_type < 3: #potrzebne do widoku 3D, ewentualne DGPS, RTK
            last_problem = f"Brak wystarczającej liczby fix_type: {msg.fix_type} < 3"
            continue
        if msg.satellites_visible <5:
            last_problem = f"Brak wystarczającej liczby satelik: {msg.satellites_visible} < 5"
            continue
        #warto także dodać eph<= 140
        if msg is None:
            continue
    return True

def check_EKF(timeout=30):
    """Czeka, aż EKF ma pewną pozycję poziomą, wysokość i prędkość."""
    EKF_REQUIRED = (
        mavlink.EKF_POS_HORIZ_ABS | mavlink.EKF_POS_VERT_ABS | mavlink.EKF_VELOCITY_HORIZ
    )
    EKF_BAD = mavlink.EKF_CONST_POS_MODE | getattr(mavlink, 'EKF_UNINITIALIZED', 0)

    last_problem = "Nie udało się uzyskać połączenia z EKF"
    deadline = timeout + time.monotonic()
    while True:
        if deadline <= time.monotonic():
            return TimeoutError(f"Nie udało się potwierdzić EKF: {last_problem} po {timeout} s")
        msg = conn.recv_match(type = 'EKF_STATUS_REPORT', blocking = True, timeout=timeout )
        if msg is None:
            continue
        ekf_ok = (
            (msg.flags & EKF_REQUIRED) == EKF_REQUIRED
            and not (msg.flags & EKF_BAD)
        )
        if ekf_ok == True:
            return True
        elif (msg.flags & EKF_REQUIRED) != EKF_REQUIRED:
            last_problem = f"Flags: {msg.flags} != EKF_REQUIRED: {EKF_REQUIRED}"
        elif msg.flags & EKF_BAD:
            last_problem = f"EKF_BAD"

def check_the_battery(timeout=10):
    msg = conn.recv_match(type='BATTERY_STATUS', blocking = True, timeout=timeout )
    if msg is None:
        raise TimeoutError("Nie udało się odczytać stanu baterii")
    return msg.battery_remaining


#być może ta funkcja nie jest potrzebna (to kontrola czujników)
# def check_sensors(timeout=5):
#     """Zwraca listę nazw czujników, które są włączone, ale niezdrowe"""
#     msg = conn.recv_match(type='SYS_STATUS', blocking=True, timeout=timeout)
#     if msg is None:
#         raise TimeoutError("Nie otrzymamo SYS_STATUS")

#     onboard_control_sensors_present = msg.onboard_control_sensors_present
#     onboard_control_sensors_enabled = msg.onboard_control_sensors_enabled
#     onboard_control_sensors_health = msg.onboard_control_sensors_health
    
def read_home_position(timeout=10):
    """Prosi autopilota o HOME_POSITION i zwraca (lat [°], lon [°], alt [m n.p.m.])."""
    msg = conn.recv_match(type='HOME_POSITION', blocking = True, timeout=timeout )
    if msg is None:
        raise TimeoutError("Nie udało się odczytać współrzędnych Home")
    return msg.latitude / 1e7, msg.longitude /1e7, msg.altitude / 1000
   
def prepare_for_start():
    if conn.motors_armed():
        raise ArmingError("Dron jest już uzbrojony")
    
    # 1: łączność
    if not check_connection():
        raise ConnectionError("Brak łączności z autopilotem")

    # 2: GPS + EKF:
    check_GPS()
    #jeśli nie zwraca erroru, tzn. że działa
    check_EKF()
    #jeśli nie zwraca erroru -> działa

    # 3: Ustawiony home -> lokalizacja startu, jak nie, to WARN w 4
    home = read_home_position()
    # 4: Aktualna pozycja drona
    start = wait_for_position()
    #jeśli nie zwraca erroru, to home jest ustawiony

    if calculate_distance(start, home) > MAX_HOME_OFFSET_M:
        if not ask_operator("Pozycja drona różni się od Home. Kontynuować?"):
            raise DifferentLandingLocation("Miejsce lądowania jest inne niż miejsce startu!")
    
    # 5: Poziom baterii powyżej progu
    battery_percentage = check_the_battery()
    if battery_percentage == -1:
        raise ValueError("Nie udało się poprawnie odczytać stanu baterii")
    if battery_percentage < MIN_BATTERY_TO_START:
        raise LowBatteryWarning("Urządzenie ma mniej niż 30% baterii")
    
    # 5: Uzbrojenie drona
    arm()


    return start, home, battery_percentage


def main():
    """Monitoruję pozycję, baterię i łączność drona """
    request_message_interval(mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1.0)
    request_message_interval(mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 1.0)
    request_message_interval(mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 1.0)
    request_message_interval(mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 1.0)
    
    try:
        start, home, battery_percentage = prepare_for_start()
    except Exception as e:
        print(f"Start przerwany: {e}")
        conn.close()
        return
    print(f"Pozycja startowa: {start}")
    
    position = None
    last_position_time = time.monotonic()
    last_print = 0.0
    try:
        while True: 
            # JEDEN odczyt na oba typy - recv_match z filtrem wyrzuca
            # wiadomości innych typów, więc dwa osobne odczyty gubiłyby danew
            msg = conn.recv_match(
                type=['GLOBAL_POSITION_INT', 'BATTERY_STATUS'],
                blocking=True,
                timeout=0.5,
            )
            now = time.monotonic()

            if msg is not None:
                msg_type = msg.get_type()
                if msg_type == 'GLOBAL_POSITION_INT':
                    position = (msg.lat / 1e7, msg.lon /1e7, msg.alt /1000)
                    last_position_time = now
                elif msg_type == 'BATTERY_STATUS':
                    ptc = msg.battery_remaining
                    if 0 <= ptc <= BATTERY_LIMIT_PCT:
                        handle_mission('battery_status')
                        break

            # brak pozycji przez POSITION_TIMEOUT_S -> sprawdzamy łącze       
            if now - last_position_time > POSITION_TIMEOUT_S:
                print(f"Brak pozycji drona od {POSITION_TIMEOUT_S} s.")
                if try_restore_connection():
                    last_position_time = time.monotonic()
                else:
                    handle_mission('heartbeat_not_responding')
                    break

            # wypisywanie pozycji co 1s
            if position is not None and now - last_print >= 1.0:
                lat, lon, alt = position
                print(f"Position: Latitude: {lat}, Longtitude: {lon}, Altitude: {alt} m")
                distance = round(calculate_distance(position, start), 2)
                print(f"Odległość od startu: {distance} m")
                last_print = time.monotonic()
    finally:             
        conn.close()


if __name__ == "__main__":
    main()

    
    
    
    