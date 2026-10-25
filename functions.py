import math as m

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

BATTERY_LIMIT_PCT = 30            # battery_remaining jest w procentach 0-100 (-1 = nieznany)
POSITION_TIMEOUT_S = 3.0          # po tylu sekundach bez pozycji sprawdzamy łącze

HEARTBEAT_CHECK_TIMEOUT_S = 5.0   # ile czekamy na heartbeat w jednej próbie
MAX_RECONNECT_TRIES = 3
EARTH_RADIUS_M = 6_371_000
MAX_HOME_OFFSET_M = 5
MIN_BATTERY_TO_START = 50
MIN_SATELLITES = 5


############################################################################
# Teraz piszemy część odpowiedzialną za lot i lądowanie 
############################################################################

def destination_point(lat, lon, bearing_deg, distance_m):
    """Liczy współrzędne punktu oddalonego o distance_m w zadanym kierunku"""
    #lat i lon mamy w kątach, bearing_deg jest w degrees
    #liczymy składowe kartezjańskie
    d_lon = distance_m * m.sin(m.radians(bearing_deg))
    d_lat = distance_m * m.cos(m.radians(bearing_deg))
    #parsujemy to na kulę ziemską
    d_angle_lat = m.degrees((d_lat  / EARTH_RADIUS_M))
    d_angle_lon = m.degrees((d_lon/ (EARTH_RADIUS_M * m.cos(m.radians(lat)))))
    lat = lat + d_angle_lat
    lon = lon + d_angle_lon
    return lat, lon

def horizontal_distance(a,b):
    return m.sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1]))

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
