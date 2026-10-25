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
MIN_SATELLITES = 5
TAKEOFF_ALT_M = 10


class PreflightWarning(Exception):
    """Wspólna klasa bazowa dla ostrzeżeń przed startem."""

class LowBatteryWarning(PreflightWarning):
    pass

class DifferentLandingLocation(PreflightWarning):
    pass

class ArmingError(Exception):
    pass

class CommandError(Exception):
    """Autopilot odrzucił polecenie."""


# tworzymy połączenie na odpowiednim porcie z conf
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
        answer = input(f"{question} (TAK/NIE): ").strip().upper()
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


def wait_for_ack(command, timeout=5):
    """Czeka na COMMAND_ACK dla danego polecenia. Przy odmowie lub braku odpowiedzi rzuca wyjątek."""
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"Brak potwierdzenia polecenia {command} po {timeout} s")
        msg = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=remaining)
        if msg is None or msg.command != command:
            continue
        if msg.result == mavlink.MAV_RESULT_IN_PROGRESS:
            continue
        if msg.result == mavlink.MAV_RESULT_ACCEPTED:
            return True
        raise CommandError(f"Polecenie {command} odrzucone, kod {msg.result}")


def set_mode(mode_name, timeout=10):
    """Ustawia tryb lotu (np. 'GUIDED') i potwierdza zmianę w HEARTBEAT."""
    mapping = conn.mode_mapping()
    if mode_name not in mapping:
        raise ValueError(f"Nieznany tryb: {mode_name}")
    mode_id = mapping[mode_name]

    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # param1: używamy trybów ArduPilota
        mode_id,                                    # param2: numer trybu
        0, 0, 0, 0, 0,
    )
    wait_for_ack(mavlink.MAV_CMD_DO_SET_MODE, timeout)

    # ACK to nie wszystko, sprawdzamy faktyczny tryb w HEARTBEAT
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        hb = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if (hb is not None
                and hb.get_srcSystem() == conn.target_system
                and hb.get_srcComponent() == conn.target_component
                and hb.custom_mode == mode_id):
            return True
    raise TimeoutError(f"Nie udało się potwierdzić zmiany trybu na {mode_name}")


def arm(timeout=10):
    """Wysyła polecenie uzbrojenia drona, czeka na ACK i potwierdza stan uzbrojenia."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
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
        msg = conn.recv_match(type=['COMMAND_ACK', 'STATUSTEXT'], blocking=True, timeout=remaining)
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
        conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if conn.motors_armed():
            return True
    raise TimeoutError("Nie udało się potwierdzić uzbrojenia")


def takeoff(timeout=30):
    """Wysyła polecenie startu i czeka, aż dron osiągnie TAKEOFF_ALT_M nad Home."""
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavlink.MAV_CMD_NAV_TAKEOFF,
        0,                  # confirmation
        0, 0, 0, 0, 0, 0,   # param1–param6 nieużywane w ArduCopterze
        TAKEOFF_ALT_M,      # param7: wysokość względem Home [m]
    )
    wait_for_ack(mavlink.MAV_CMD_NAV_TAKEOFF)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg is None:
            continue
        alt = msg.relative_alt / 1000       # mm → m, względem Home
        if alt >= TAKEOFF_ALT_M * 0.95:     # tolerujemy 5% poniżej celu
            print(f"Osiągnięto wysokość {alt:.1f} m")
            return True
    raise TimeoutError(f"Dron nie wzbił się na wysokość {TAKEOFF_ALT_M} m po {timeout} s")


def start_flight():
    """Tryb GUIDED → uzbrojenie → start. Wywoływać dopiero po prepare_for_start()."""
    set_mode('GUIDED')
    arm()
    try:
        takeoff()
    except Exception:
        # dron jest uzbrojony, ale jeszcze na ziemi, więc rozbrojenie jest bezpieczne
        conn.arducopter_disarm()
        raise


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


def check_connection(timeout=HEARTBEAT_CHECK_TIMEOUT_S):
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
    """Próbuje n razy złapać heartbeat"""
    for attempt in range(1, MAX_RECONNECT_TRIES + 1):
        if check_connection():
            print("Heartbeat OK - łączność jest")
            return True
        print(f"Próba {attempt}/{MAX_RECONNECT_TRIES}: brak heartbeat")
    return False


def fallback_return_plan():
    pass


def handle_mission(reason):
    """Na podstawie błędów łączności / niskiego poziomu baterii decyduje, czy przerwać misję"""
    match reason:
        case 'battery_status':
            # bateria poniżej BATTERY_LIMIT_PCT
            fallback_return_plan()
        case 'heartbeat_not_responding':
            pass
        case 'sys_error':
            pass
        case 'connection_error':
            pass


def check_GPS(timeout=30):
    """Czeka, aż GPS ma fix 3D i wystarczająco satelitów. Inaczej rzuca wyjątek."""
    deadline = time.monotonic() + timeout
    last_problem = "brak wiadomości GPS_RAW_INT"
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"Brak gotowości GPS po {timeout} s: {last_problem}")
        msg = conn.recv_match(type='GPS_RAW_INT', blocking=True, timeout=remaining)
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


def check_EKF(timeout=30):
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
        msg = conn.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=remaining)
        if msg is None:
            continue
        if (msg.flags & EKF_REQUIRED) != EKF_REQUIRED:
            last_problem = f"brak wymaganych flag (flags = {bin(msg.flags)})"
        elif msg.flags & EKF_BAD:
            last_problem = f"EKF w złym stanie (flags = {bin(msg.flags)})"
        else:
            return True


def check_the_battery(timeout=10):
    msg = conn.recv_match(type='BATTERY_STATUS', blocking=True, timeout=timeout)
    if msg is None:
        raise TimeoutError("Nie udało się odczytać stanu baterii")
    return msg.battery_remaining


def read_home_position(timeout=10):
    """Prosi autopilota o HOME_POSITION i zwraca (lat [°], lon [°], alt [m n.p.m.])."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavlink.MAVLINK_MSG_ID_HOME_POSITION,
        0, 0, 0, 0, 0, 0,
    )
    msg = conn.recv_match(type='HOME_POSITION', blocking=True, timeout=timeout)
    if msg is None:
        raise TimeoutError("Nie udało się odczytać współrzędnych Home")
    return msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1000


def prepare_for_start():
    """Same sprawdzenia przed startem - nic tu nie uzbraja drona."""
    if conn.motors_armed():
        raise ArmingError("Dron jest już uzbrojony")

    # 1: łączność
    if not check_connection():
        raise ConnectionError("Brak łączności z autopilotem")

    # 2: GPS + EKF (każda funkcja rzuca wyjątek, jeśli nie są gotowe)
    check_GPS()
    check_EKF()

    # 3: punkt Home i aktualna pozycja drona
    home = read_home_position()
    start = wait_for_position()
    if calculate_distance(start, home) > MAX_HOME_OFFSET_M:
        if not ask_operator("Pozycja drona różni się od Home. Kontynuować?"):
            raise DifferentLandingLocation("Pozycja drona różni się od punktu Home")

    # 4: poziom baterii powyżej progu
    battery_percentage = check_the_battery()
    if battery_percentage == -1:
        raise ValueError("Nie udało się poprawnie odczytać stanu baterii")
    if battery_percentage < MIN_BATTERY_TO_START:
        raise LowBatteryWarning(
            f"Bateria {battery_percentage}%, minimum do startu to {MIN_BATTERY_TO_START}%"
        )

    return start, home, battery_percentage


def main():
    """Monitoruje pozycję, baterię i łączność drona"""
    request_message_interval(mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1.0)
    request_message_interval(mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 1.0)
    request_message_interval(mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 1.0)
    request_message_interval(mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 1.0)

    try:
        start, home, battery_percentage = prepare_for_start()
        print(f"Pozycja startowa: {start}")
        start_flight()      # GUIDED → uzbrojenie → start
    except Exception as e:
        print(f"Start przerwany: {e}")
        conn.close()
        return

    position = None
    last_position_time = time.monotonic()
    last_print = 0.0
    try:
        while True:
            # JEDEN odczyt na oba typy - recv_match z filtrem wyrzuca
            # wiadomości innych typów, więc dwa osobne odczyty gubiłyby dane
            msg = conn.recv_match(
                type=['GLOBAL_POSITION_INT', 'BATTERY_STATUS'],
                blocking=True,
                timeout=0.5,
            )
            now = time.monotonic()

            if msg is not None:
                msg_type = msg.get_type()
                if msg_type == 'GLOBAL_POSITION_INT':
                    position = (msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000)
                    last_position_time = now
                elif msg_type == 'BATTERY_STATUS':
                    pct = msg.battery_remaining
                    if 0 <= pct <= BATTERY_LIMIT_PCT:
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

            # wypisywanie pozycji co 1 s
            if position is not None and now - last_print >= 1.0:
                lat, lon, alt = position
                print(f"Position: Latitude: {lat}, Longitude: {lon}, Altitude: {alt} m")
                distance = round(calculate_distance(position, start), 2)
                print(f"Odległość od startu: {distance} m")
                last_print = time.monotonic()
    finally:
        conn.close()


if __name__ == "__main__":
    main()
