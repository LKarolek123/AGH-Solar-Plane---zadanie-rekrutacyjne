
# Dopasuj do przekierowania komunikacji w SITL.
MAVLINK_CONNECTION = "tcp:127.0.0.1:5762"
MAVLINK_DIALECT = "ardupilotmega"
HEARTBEAT_TIMEOUT_S = 15
SOURCE_SYSTEM = 245

# Przykładowe wartości do symulacji.
TAKEOFF_ALTITUDE_M = 20
SEARCH_ALTITUDE_M = 20
MISSION_TIMEOUT_S = 600

# Testowy czujnik i mechanizm zrzutu.
DETECTION_SOURCE = "data/detections.json"
PAYLOAD_SIMULATED = True