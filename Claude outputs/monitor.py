import os
os.environ['MAVLINK20'] = '1'   # musi być ustawione PRZED importem pymavlink

import math
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from conf import MAVLINK_CONNECTION, MAVLINK_DIALECT, SOURCE_SYSTEM, HEARTBEAT_TIMEOUT_S

BATTERY_LIMIT_PCT = 55            # battery_remaining jest w procentach 0-100 (-1 = nieznany)
POSITION_TIMEOUT_S = 3.0          # po tylu sekundach bez pozycji sprawdzamy łącze
HEARTBEAT_CHECK_TIMEOUT_S = 2.0   # ile czekamy na heartbeat w jednej próbie
MAX_RECONNECT_TRIES = 3
EARTH_RADIUS_M = 6_371_000

# tworzymy połączenie na odpowiednim porcie z conf
conn = mavutil.mavlink_connection(
    MAVLINK_CONNECTION,
    dialect=MAVLINK_DIALECT,
    source_system=SOURCE_SYSTEM,
)

heartbeat = conn.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S)
if heartbeat is None:
    raise TimeoutError('Brak heartbeat autopilota')
print(f"Połączono: system {conn.target_system}, komponent {conn.target_component}")


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


def wait_for_position(timeout=10):
    """Zwraca (lat [°], lon [°], alt [m n.p.m.]) z pierwszego GLOBAL_POSITION_INT."""
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
    mean_lat = math.radians((lat1 + lat2) / 2)
    dx = math.radians(lon2 - lon1) * math.cos(mean_lat) * EARTH_RADIUS_M
    dy = math.radians(lat2 - lat1) * EARTH_RADIUS_M
    dz = alt2 - alt1
    return round(math.sqrt(dx * dx + dy * dy + dz * dz), 3)


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
    """Do MAX_RECONNECT_TRIES prób złapania heartbeatu."""
    for attempt in range(1, MAX_RECONNECT_TRIES + 1):
        if check_connection():
            print("Heartbeat OK - łączność jest")
            return True
        print(f"Próba {attempt}/{MAX_RECONNECT_TRIES}: brak heartbeat")
    return False


def fallback_return_plan():
    pass


def handle_mission(reason):
    """Main function used to decide: fallback return or continue the mission"""
    match reason:
        case 'battery_status':
            fallback_return_plan()
        case 'heartbeat_not_responding':
            pass
        case 'sys_error':
            pass
        case 'connection_error':
            pass


def main():
    """Monitoruje pozycję, baterię i łączność drona."""
    request_message_interval(mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1.0)
    request_message_interval(mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 1.0)

    start = wait_for_position()
    print(f"Pozycja startowa: {start}")

    position = None
    last_print = 0.0
    last_position_time = time.monotonic()

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
                        print(f"Bateria {pct}% - powrót")
                        handle_mission('battery_status')
                        break

            # brak pozycji przez POSITION_TIMEOUT_S -> sprawdzamy łącze
            if now - last_position_time > POSITION_TIMEOUT_S:
                print(f"Brak pozycji drona od {POSITION_TIMEOUT_S:.0f} s")
                if try_restore_connection():
                    last_position_time = time.monotonic()
                else:
                    handle_mission('heartbeat_not_responding')
                    break

            # wypisywanie co 1 s
            if position is not None and now - last_print >= 1.0:
                last_print = now
                lat, lon, alt = position
                print(f"Latitude: {lat:.7f}, longitude: {lon:.7f}, altitude: {alt:.2f} m")
                print(f"Odległość od startu: {calculate_distance(start, position)} m")
    finally:
        conn.close()


if __name__ == "__main__":
    main()
