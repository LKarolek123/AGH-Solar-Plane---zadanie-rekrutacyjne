from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega
import os
from conf import *
os.environ['MAVLINK20'] = '1'
import time
import math as m

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

#okreslenie pozycji drona w chwili startu:
msg = conn.recv_match(type='Global_position_INT', blocking=True, timeout=3)
START_LAT = msg.latitude / 1e7
START_LON = msg.longitute / 1e7
START_ALT = msg.alt / 1000


def calculate_distance(lat, lon, alt):
    """Calculates the distance between drone and the start point"""
    distance_square = m.pow(START_LAT-lat, 2) + m.pow(START_LON-lon, 2) + m.pow(START_ALT-alt, 2)
    return round(m.sqrt(distance_square), 3)

def check_connection():
    """Used when we are not sure that connection is stable = when we don't have system logs"""

    # Identyfikatory ustalone wcześniej podczas połączenia.
    drone_system = conn.target_system
    drone_component = conn.target_component

    last_heartbeat = None
    heartbeat_timeout = 5.0 


    
    msg = conn.recv_match(blocking=True, timeout=0.5)

    if (
        msg is not None
        and msg.get_type() == "HEARTBEAT"
        and msg.get_srcSystem() == drone_system
        and msg.get_srcComponent() == drone_component
    ):
        last_heartbeat = time.monotonic()

    connected = (
        last_heartbeat is not None
        and time.monotonic() - last_heartbeat < heartbeat_timeout
    )



    return("Heartbeat OK" if connected else "Brak aktualnego heartbeat")
    



def fallback_return_plan():
    pass

def handle_mission(reason):
    """Main function used to decide: fallback return or continue the mission"""
    match reason:
        case 'battery_status':
            #battery status below 56%
            fallback_return_plan()
        case 'heartbeat_not_responding':
            pass
        case 'sys_error':
            pass
        case 'connection_error':
            pass


def main():
    """Main function usedd to monitor drone's connection& """
    last = 0
    while True:        
        #sprawdzanie aktualnej pozycji drona co 1s
        msg = conn.recv_match(type='Global_position_INT', blocking=True, timeout=3)
        now = time.monotonic()
        if now - last >= 1.0:
            last = now
            lat = msg.latitude / 1e7
            lon = msg.longitute / 1e7
            alt = msg.alt / 1000
        if msg is None :
            print("Nie otrzymano pozycji drona w ciągu 3 sekund")
            # if so, we need to check the connection
            

            #3 razy probujemy przywrocic polaczenie - jesli nie dziala, odsylamy do handle_mission
            connection_checked_times = 0
            interval = 1.0
            success = False
            while connection_checked_times<3:
                try:
                    status = check_connection()
                    if status == "Heartbeat ok":
                        success = True
                        break
                    else:
                        continue
                except TimeoutError:
                    connection_checked_times +=1
                    if connection_checked_times == 3:
                        break
                    time.sleep(interval)
            if success == False:
                handle_mission('heartbeat_not_responding')
        else:
            print(f"Latitude: {lat}, longtitude: {lon}, altitude: {alt}")
            print(calculate_distance(lat, lon, alt))

        #sprawdzanie statusu baterii 
        battery_status = conn.recv_match(type='BATTERY_STATUS', blocking=True, timeout=15)
        if battery_status.battery_remaining <= 0.55:
            handle_mission('battery_status')
            break

            


                 
    conn.close()

if __name__ == "__main__":
    main()

    
    
    
    