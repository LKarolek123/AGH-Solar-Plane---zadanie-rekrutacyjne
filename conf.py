"""Konfiguracja misji. Wszystkie stałe trzymamy w jednym miejscu."""

# --- Połączenie z autopilotem ---------------------------------------------
# Dopasuj do przekierowania komunikacji w SITL.
MAVLINK_CONNECTION = "tcp:127.0.0.1:5762"
MAVLINK_DIALECT = "ardupilotmega"
HEARTBEAT_TIMEOUT_S = 15
SOURCE_SYSTEM = 245

POSITION_TIMEOUT_S = 3.0          # po tylu sekundach bez pozycji sprawdzamy łącze
HEARTBEAT_CHECK_TIMEOUT_S = 5.0   # ile czekamy na heartbeat w jednej próbie
MAX_RECONNECT_TRIES = 3

# --- Przykładowe wartości do symulacji (pozostałość po wcześniejszej wersji) ---
SEARCH_ALTITUDE_M = 20
MISSION_TIMEOUT_S = 600
MAX_MISSION_DISTANCE = 20
MISSION_ANGLE_DEGREES = 45

# Testowy czujnik i mechanizm zrzutu.
DETECTION_SOURCE = "data/detections.json"
PAYLOAD_SIMULATED = True

# --- Cel misji: Miasteczko Studenckie AGH ----------------------------------
TARGET_LATITUDE = 50.068964
TARGET_LONGITUDE = 19.906171

# --- Start i przygotowanie -------------------------------------------------
TAKEOFF_ALTITUDE_M = 12           # wysokość startu i całego przelotu (względem Home)
MIN_BATTERY_TO_START = 50         # [%]
MAX_HOME_OFFSET_M = 5             # dopuszczalna różnica pozycja drona - Home przed startem
MIN_SATELLITES = 5
EARTH_RADIUS_M = 6_371_000

# --- Przelot ---------------------------------------------------------------
CRUISE_SPEED_MPS = 10.0           # v_d - prędkość przelotu do celu i z powrotem
TARGET_REACHED_TOLERANCE_M = 1.5  # promień, w którym uznajemy cel za osiągnięty
HOME_REACHED_TOLERANCE_M = 1.0
LAND_DESCENT_SPEED_MPS = 0.5      # konserwatywna prędkość opadania do szacowania lądowania
SETPOINT_RATE_HZ = 5              # jak często wysyłamy punkty przy śledzeniu trajektorii

# --- Koło pomiarowe (szacowanie średniego poboru prądu) --------------------
CIRCLE_RADIUS_M = 5.0
CIRCLE_SPEED_MPS = 2.0            # prędkość liniowa na okręgu
CIRCLE_LAPS = 1
MIN_CURRENT_SAMPLES = 5           # minimum pomiarów prądu, żeby ufać średniej

# --- Bateria ---------------------------------------------------------------
BATTERY_RESERVE_FRACTION = 0.10   # bezwzględny zapas: 10% pojemności zawsze zostaje
BATTERY_CAPACITY_MAH_FALLBACK = 3300  # gdy nie uda się odczytać BATT_CAPACITY z autopilota
BATTERY_LIMIT_PCT = 20            # twardy próg awaryjny (niezależny od budżetu energii)
ENERGY_SAFETY_FACTOR = 1.2        # mnożnik na zapotrzebowanie przy powrocie (wiatr, przyspieszanie)

# --- Muzyka i głośnik ------------------------------------------------------
MUSIC_FILE = "Kukurin.mp3"
MUSIC_DURATION_S = 60             # "Kukurin na full przez minutę"
SPEAKER_BACKEND = "pygame"        # "pygame" (dźwięk z komputera) albo "simulated" (tylko log)
SPEAKER_CURRENT_A = 2.0           # średni dodatkowy pobór głośnika na maksymalnej głośności
SPEAKER_VALIDATION_S = 5          # po tylu sekundach muzyki weryfikujemy pobór i odtwarzanie
MUSIC_RESTART_ATTEMPTS = 1        # ile razy próbujemy ponownie uruchomić muzykę po awarii

# --- Spirala: rzut f(x) = (-2)^x na płaszczyznę zespoloną -------------------
SPIRAL_SPEED_MPS = 0.2            # prędkość liniowa po spirali (20 cm/s)
SPIRAL_START_RADIUS_M = 0.1       # spirala logarytmiczna nie zaczyna się w zerze
SPIRAL_MIN_DIAMETER_M = 5.0       # kryterium sukcesu misji

# --- Logi ------------------------------------------------------------------
POSITION_LOG_INTERVAL_S = 1.0
BATTERY_LOG_INTERVAL_S = 5.0
ETA_LOG_INTERVAL_S = 5.0
