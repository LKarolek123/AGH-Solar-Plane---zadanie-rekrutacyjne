from vehicle import Vehicle


from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from conf import *

from errors import *
from functions import *
import sys

def emergency_return(vehicle):
    """Próbuje bezpiecznie sprowadzić drona: najpierw RTL, a jeśli się nie uda, LAND."""
    for mode in ('RTL', 'LAND'):
        try:
            vehicle.set_mode(mode)
            print(f"Przełączono w tryb {mode}")
            return True
        except Exception as e:
            print(f"Nie udało się przełączyć w {mode}: {e}")
    print("UWAGA: dron jest w powietrzu bez kontroli programu - przejmij go w Mission Plannerze!")
    return False

def main():
    conn = mavutil.mavlink_connection(
        MAVLINK_CONNECTION,
        dialect=MAVLINK_DIALECT,
        source_system=SOURCE_SYSTEM,
        autoreconnect=True,
    )
    vehicle = None
    try:
        heartbeat = conn.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S)
        if heartbeat is None:
            raise TimeoutError('Brak heartbeat autopilota')
        conn.target_component = heartbeat.get_srcComponent()

        vehicle = Vehicle(conn)
        vehicle.request_message_interval(mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1.0)
        vehicle.request_message_interval(mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 1.0)
        vehicle.request_message_interval(mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 1.0)
        vehicle.request_message_interval(mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 1.0)
        vehicle.request_message_interval(mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 1.0)

        start, home, battery_status = vehicle.prepare_for_start()
        vehicle.home = home
        vehicle.start = start
        vehicle.battery_status = battery_status
        vehicle.start_flight()
        lat, lon = destination_point(vehicle.start[0], vehicle.start[1], MISSION_ANGLE_DEGREES, MAX_MISSION_DISTANCE)
        target = (TARGET_LATITUDE, TARGET_LONGITUDE, TAKEOFF_ALTITUDE_M)
        vehicle.goto_position(target)
        vehicle.wait_until_reached(target, 1.5, 60)
        # powrót nad Home i lądowanie

        print("Misja zakończona powodzeniem")
        vehicle.return_home(TAKEOFF_ALTITUDE_M)
        vehicle.land()
        vehicle.check_disarm_confirmation()
        print("Misja zakończona sukcesem!")
        return 0

    except PreflightWarning as e:
        # słaba bateria, pozycja różna od Home, odmowa operatora - dron na ziemi
        print(f"Start wstrzymany: {e}")
        return 1

    except KeyboardInterrupt:
        print("Przerwano przez operatora (Ctrl+C)")
        if vehicle is not None and conn.motors_armed():
            emergency_return(vehicle)
        return 2

    except Exception as e:
        print(f"Misja przerwana: {type(e).__name__}: {e}")
        if vehicle is not None and conn.motors_armed():
            emergency_return(vehicle)
        return 1

    finally:
        conn.close()


if __name__ == "__main__":
    sys.exit(main())
