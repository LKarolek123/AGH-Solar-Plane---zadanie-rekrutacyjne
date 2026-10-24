# pymavlink i ardupilotmega — dokumentacja do misji ratunkowej

Data opracowania: 24.10.2026. Cel: wyszukanie osób, lokalizacja celu, dostarczenie zaopatrzenia, powrót i lądowanie.

## Zakres i status weryfikacji

To katalog API istotnego dla tej misji, a nie lista wszystkich funkcji całego pakietu. Nazwy zestawiono z oficjalnym kodem i dokumentacją podlinkowaną na końcu. Nie wykonano testu połączenia z SITL. W środowisku `.venv` projektu podczas sprawdzenia nie było zainstalowanego pymavlink; requirements.txt zawiera `pymavlink` bez wersji. Dokumentacja internetowa i gałąź master mogą różnić się od później zainstalowanego wydania. Przed implementacją należy przypiąć wersję pakietu i firmware SITL.

Przyjęty wariant: wielowirnikowiec ArduCopter z GPS, kamera skierowana w dół i mechanizm zwalniający paczkę. Dla ArduPlane trzeba zmienić model dolotu, startu, lądowania i zrzutu; samolot nie zawisa.

`ardupilotmega` jest generowanym dialektem MAVLink w pymavlink, a nie biblioteką wykrywania osób. Zawiera również wiadomości z common. Typowy import dla MAVLink 2: `from pymavlink.dialects.v20 import ardupilotmega`. W aplikacji korzystającej z mavutil należy ustawić protokół przed importem i jawnie wybrać dialekt połączenia.

## 1. Funkcje narzędziowe pymavlink

Poniżej `conn` oznacza obiekt zwrócony przez `mavutil.mavlink_connection()`. Metody wysyłające wiadomości są dostępne przez `conn.mav`.

| Funkcja / metoda | Zastosowanie w projekcie |
|---|---|
| `mavutil.mavlink_connection(...)` | Utworzenie połączenia; adres i port wynikają z konfiguracji SITL. |
| `mavutil.set_dialect(...)` | Wybór definicji wiadomości; zwykle wystarczy parametr dialect połączenia. |
| `conn.wait_heartbeat(...)` | Oczekiwanie na obecność autopilota; stosuj timeout. |
| `conn.recv_match(...)` | Odbiór wybranych typów wiadomości; obsłuż wynik None. |
| `conn.recv_msg()` | Odbiór pojedynczej wiadomości bez filtra typu. |
| `conn.mode_mapping()` | Mapa nazw trybów do identyfikatorów dla pojazdu. |
| `conn.set_mode(...)` | Żądanie zmiany trybu; sprawdzaj rzeczywisty tryb w telemetrii. |
| `conn.arducopter_arm()` | Żądanie uzbrojenia ArduCoptera. |
| `conn.arducopter_disarm()` | Żądanie rozbrojenia; nie używaj jako sposobu przerywania lotu. |
| `conn.motors_armed()` | Odczyt flagi uzbrojenia; nie jest pomiarem obrotów silników. |
| `conn.motors_armed_wait()` / `motors_disarmed_wait()` | Pomocnicze oczekiwanie; sprawdź obsługę timeout w wybranej wersji, ewentualnie użyj własnej ograniczonej czasowo pętli. |
| `conn.time_since(...)` | Wiek ostatniej wiadomości danego typu. |
| `conn.param_fetch_one(...)` / `param_fetch_all()` | Żądanie parametrów konfiguracji. |
| `conn.param_set_send(...)` | Zmiana parametru; potwierdź odczytem, nie zmieniaj automatycznie ustawień bezpieczeństwa. |
| `conn.close()` | Zamknięcie połączenia. |

Wiadomości: `msg.get_type()`, `get_srcSystem()`, `get_srcComponent()`, `to_dict()` pomagają filtrować nadawcę i zapisywać logi. `conn.messages` jest pamięcią ostatnich wiadomości, nie gwarancją ich aktualności.

Źródło: [mavutil.py](https://github.com/ArduPilot/pymavlink/blob/master/mavutil.py).

## 2. Wysyłanie wiadomości i poleceń

`MAV_CMD_*` to stałe identyfikujące polecenia, a nie funkcje Pythona. Polecenie przesyła się np. przez `command_long_send()`. Dla każdej pozycji sprawdź parametry w dokumentacji danego polecenia oraz wsparcie firmware.

| Metoda `conn.mav.*` | Cel |
|---|---|
| `command_long_send(...)` | Polecenie z siedmioma parametrami. |
| `command_int_send(...)` | Polecenie z jawną ramą współrzędnych i polami pozycji całkowitoliczbowej. |
| `heartbeat_send(...)` | Ogłaszanie obecności aplikacji, jeśli przewiduje to konfiguracja systemu. |
| `ping_send(...)` | Diagnostyka komunikacji. |
| `timesync_send(...)` | Element synchronizacji czasu; wymaga implementacji obsługi odpowiedzi. |
| `set_position_target_global_int_send(...)` | Zadanie pozycji/prędkości w układzie globalnym. |
| `set_position_target_local_ned_send(...)` | Zadanie pozycji/prędkości w układzie lokalnym NED. |
| `set_attitude_target_send(...)` | Sterowanie orientacją; opcjonalne, poza podstawową misją waypointową. |
| `param_request_read_send(...)` / `param_request_list_send(...)` | Odczyt parametrów. |
| `param_set_send(...)` | Zapis parametru przez protokół parametrów. |

Przy sterowaniu GUIDED `type_mask` określa aktywne pola. Globalne lat/lon mają zwykle skalę 1e7; jednostki wysokości zależą od wiadomości. W NED oś Z jest skierowana w dół. Setpointy prędkości wymagają regularnego ponawiania zgodnie z zachowaniem ArduCoptera. Nie zakładaj, że wysłanie setpointu daje COMMAND_ACK.

Źródła: [GUIDED](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html), [definicje common](https://mavlink.io/en/messages/common.html).

## 3. Polecenia potrzebne w misji

| Stała | Rola |
|---|---|
| `MAV_CMD_COMPONENT_ARM_DISARM` | Uzbrojenie / rozbrojenie. |
| `MAV_CMD_NAV_TAKEOFF` | Start. |
| `MAV_CMD_NAV_WAYPOINT` | Punkt trasy. |
| `MAV_CMD_NAV_LOITER_TIME` / `MAV_CMD_NAV_LOITER_UNLIM` | Oczekiwanie w ramach misji. |
| `MAV_CMD_NAV_RETURN_TO_LAUNCH` | Powrót. |
| `MAV_CMD_NAV_LAND` | Lądowanie. |
| `MAV_CMD_DO_CHANGE_SPEED` | Zmiana prędkości. |
| `MAV_CMD_CONDITION_YAW` | Ustawienie kierunku, gdy obsługiwane. |
| `MAV_CMD_DO_SET_ROI_LOCATION` / `MAV_CMD_DO_SET_ROI_NONE` | Wskazanie / usunięcie obszaru zainteresowania; zależy od konfiguracji kamery/gimbala. |
| `MAV_CMD_SET_MESSAGE_INTERVAL` / `MAV_CMD_REQUEST_MESSAGE` | Częstotliwość telemetrii / pojedyncze żądanie. |
| `MAV_CMD_DO_SET_SERVO` | Sterowanie serwem zwalniacza. |
| `MAV_CMD_DO_SET_RELAY` | Sterowanie przekaźnikiem, jeśli taki mechanizm zastosowano. |
| `MAV_CMD_DO_GRIPPER` | Obsługa skonfigurowanego chwytaka. |
| `MAV_CMD_IMAGE_START_CAPTURE` / `MAV_CMD_IMAGE_STOP_CAPTURE` | Rejestracja zdjęć na zgodnej kamerze MAVLink. |
| `MAV_CMD_DO_SET_CAM_TRIGG_DIST` | Wyzwalanie kamery według przebytego dystansu. |

Źródła: [polecenia misji](https://ardupilot.org/dev/docs/common-mavlink-mission-command-messages-mav_cmd.html), [pełny rejestr poleceń](https://mavlink.io/en/messages/common.html#MAV_CMD). Dostępność w dialekcie nie oznacza obsługi w danym firmware ani dowolnym trybie lotu.

## 4. Telemetria: co odbierać

| Wiadomość | Zastosowanie |
|---|---|
| `HEARTBEAT` | Obecność, tryb i uzbrojenie. |
| `SYS_STATUS` / `BATTERY_STATUS` | Stan systemu i energii. |
| `GPS_RAW_INT` / `GLOBAL_POSITION_INT` | Jakość GPS i estymowana pozycja globalna. |
| `LOCAL_POSITION_NED` / `ATTITUDE` / `ATTITUDE_QUATERNION` | Pozycja lokalna i orientacja do lokalizacji celu. |
| `HOME_POSITION` | Punkt odniesienia powrotu. |
| `EXTENDED_SYS_STATE` | Informacja m.in. o stanie lądowania. |
| `STATUSTEXT` / `COMMAND_ACK` | Diagnostyka i wynik polecenia. |
| `DISTANCE_SENSOR` / `OBSTACLE_DISTANCE` | Odległość od powierzchni lub przeszkód, jeśli źródło jest skonfigurowane. |
| `SERVO_OUTPUT_RAW` | Wysterowanie wyjść; nie potwierdza fizycznego odczepienia paczki. |
| `CAMERA_IMAGE_CAPTURED` / `CAMERA_CAPTURE_STATUS` | Rejestracja obrazu przez zgodną kamerę. |
| `GIMBAL_DEVICE_ATTITUDE_STATUS` | Orientacja zgodnego gimbala. |
| `SYSTEM_TIME` / `TIMESYNC` | Powiązanie czasów pomiarów. |

Rozszerzenia ardupilotmega, opcjonalnie: `EKF_STATUS_REPORT` (estymator), `RANGEFINDER` (starszy format dalmierza), `AHRS2` / `AHRS3` (dodatkowe dane estymacji), `HWSTATUS` (diagnostyka sprzętu), `MOUNT_STATUS` (starszy format montażu), `FENCE_STATUS` (ogrodzenie). Preferuj wspólne, aktualne wiadomości tam, gdzie wystarczają.

Źródła: [common](https://mavlink.io/en/messages/common.html), [ardupilotmega](https://mavlink.io/en/messages/ardupilotmega.html). ArduPilot ostrzega, że referencja dialektu może odstawać od [jego XML](https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/ardupilotmega.xml).

## 5. Planowanie i przesyłanie trasy

Metody `conn.mav`: `mission_count_send`, `mission_item_int_send`, `mission_request_list_send`, `mission_request_int_send`, `mission_ack_send`, `mission_clear_all_send`, `mission_set_current_send`.

Obsługiwane wiadomości odpowiedzi: `MISSION_REQUEST_INT`, `MISSION_COUNT`, `MISSION_ITEM_INT`, `MISSION_ACK`, `MISSION_CURRENT`, `MISSION_ITEM_REACHED`. Jeśli firmware zażąda starszego formatu `MISSION_REQUEST`, potrzebna jest zgodna obsługa zamiast bezwarunkowego wysłania innego typu.

Upload jest wymianą żądań i odpowiedzi: liczba elementów → żądane indeksy → elementy → końcowy ACK. Obsłuż powtórzone żądania, timeout i nieprawidłowy indeks. `MISSION_ACK` dotyczy transferu, a nie wykonania całej trasy.

`mavwp.MAVWPLoader`: `load`, `save`, `add`, `insert`, `set`, `remove`, `clear`, `count`, `wp`/`item`, `reindex` pomagają zarządzać listą i plikami waypointów. Sam loader nie realizuje kompletnego uploadu do autopilota.

Źródła: [protokół misji](https://mavlink.io/en/services/mission.html), [mavwp.py](https://github.com/ArduPilot/pymavlink/blob/master/mavwp.py).

## 6. Potwierdzenia, błędy i zrzut

Obsługuj `COMMAND_ACK.command`, `result` i ewentualny postęp. Rozróżniaj `MAV_RESULT_ACCEPTED`, `IN_PROGRESS`, `TEMPORARILY_REJECTED`, `DENIED`, `UNSUPPORTED`, `FAILED`, `CANCELLED`. Dopasowuj odpowiedź do polecenia i nadawcy. Nie wysyłaj równolegle nierozróżnialnych egzemplarzy tego samego polecenia.

ACK nie zastępuje obserwacji wyniku: po starcie sprawdź wysokość, po zmianie trybu HEARTBEAT, po lądowaniu stan przyziemienia. Utrata ACK zwalniacza nie upoważnia do bezwarunkowego ponowienia fizycznego zrzutu.

Źródło: [Command Protocol](https://mavlink.io/en/services/command.html).

Konfiguracja serwa wymaga dobrania kanału, `SERVOx_FUNCTION`, zakresu PWM i pozycji zamkniętej/otwartej do mechanizmu. Dla `MAV_CMD_DO_SET_SERVO` param1 oznacza numer wyjścia, param2 PWM. Nie kopiuj arbitralnego kanału ani PWM na rzeczywisty sprzęt.

Źródło: [Move a Servo](https://ardupilot.org/dev/docs/mavlink-move-servo.html).

Projektowy warunek zrzutu: potwierdzony cel, aktualna lokalizacja, poprawny obszar dostawy, dopuszczalna prędkość i wysokość, dostępny ładunek. Punkt dostawy powinien być odsunięty od osoby o zaplanowany margines. Rejestruj identyfikator dostawy i stan `READY → RELEASE_REQUESTED → RELEASE_CONFIRMED` lub `UNKNOWN`. Fizyczne potwierdzenie wymaga czujnika mechanizmu; w SITL potrzebny jest jawny model zastępczy.

## 7. Czego ta biblioteka nie zrobi za aplikację

- Detekcja człowieka nie dowodzi, że potrzebuje pomocy. Na potrzeby zadania jawnie przyjmij regułę kwalifikacji, np. oznaczony obszar i sygnał osoby, albo zasymuluj status potrzebującego.
- Obraz kamery USB/CSI pobieraj przez właściwy sterownik lub bibliotekę obrazu. Sterowanie kamerą MAVLink nie jest automatycznie strumieniem obrazu.
- Współrzędne piksela nie są GPS osoby. Potrzebujesz kalibracji kamery, orientacji kamery i pojazdu, pozycji, wysokości nad terenem oraz czasowego dopasowania pomiarów. Wynik powinien mieć oszacowanie niepewności.
- `LANDING_TARGET` dotyczy celu do lądowania; `FOLLOW_TARGET` śledzenia celu. Nie są gotową reprezentacją osoby potrzebującej pomocy ani poleceniem precyzyjnego zrzutu.
- Plan przeszukiwania, deduplikację osób, wybór kolejności pomocy, bilans energii i zapasów oraz decyzje o przerwaniu opracowujesz w aplikacji.

Proponowane własne interfejsy: `Detector.detect`, `TargetLocalizer.locate`, `SearchPlanner.plan`, `MissionController.step`, `VehicleAdapter.takeoff/goto/return_home/land`, `PayloadController.release_once`, `EventRecorder.record`. To nazwy projektowe, nie istniejące funkcje pymavlink.

## 8. Minimalny start połączenia, bez uruchamiania napędu

```python
import os
os.environ['MAVLINK20'] = '1'

from pymavlink import mavutil

conn = mavutil.mavlink_connection(
    'udpin:0.0.0.0:14550',  # ustaw zgodnie z przekierowaniem SITL
    dialect='ardupilotmega',
    source_system=245,      # wybierz unikalny identyfikator w swojej sieci
)
try:
    heartbeat = conn.wait_heartbeat(timeout=15)
    if heartbeat is None:
        raise TimeoutError('Brak heartbeat autopilota')
    print(conn.target_system, conn.target_component)
finally:
    conn.close()
```

W docelowej aplikacji zweryfikuj typ/nadawcę heartbeat i użyj jednego odbiornika rozdzielającego wiadomości do modułów. Konkurujące wywołania recv_match mogą odebrać odpowiedź przeznaczoną dla innej części systemu. Mission Planner i program powinny otrzymywać osobne przekierowane strumienie zamiast przypadkowo konkurować o ten sam port.

## 9. Testy do dokumentacji rekrutacyjnej

| Scenariusz | Oczekiwany wynik |
|---|---|
| Osoba wykryta, poprawna lokalizacja | Dolot do punktu dostawy, jeden zrzut, powrót i lądowanie. |
| Brak detekcji | Zakończenie przeszukiwania lub przekroczenie budżetu czasu; powrót bez zrzutu. |
| Detekcja powtórzona | Jedna dostawa dla tej samej osoby. |
| Błędna lub nieaktualna pozycja celu | Brak zgody na zrzut, ponowny pomiar lub przerwanie. |
| Odrzucone polecenie | Zapis przyczyny i kontrolowane przejście do procedury awaryjnej. |
| Utracone potwierdzenie zwolnienia paczki | Stan UNKNOWN i brak ślepego ponowienia zrzutu. |
| Brak heartbeat / niski poziom energii | Zachowanie zgodne z polityką aplikacji i skonfigurowanymi failsafe autopilota. |
| Cel poza obszarem misji / brak paczek | Pominięcie dostawy, zdarzenie w logu. |

SITL: ruch, tryby, telemetria, polecenia, trasa i parametry. Osobno: powtarzalne nagrania lub pliki detekcji oraz model mechanizmu i wyniku dostawy. Nie przedstawiaj zmiany PWM jako dowodu trafienia paczki. Wyniki testów dopisz dopiero po wykonaniu; powyżej zapisano plan i oczekiwania.

## 10. Źródła i utrzymanie

- [pymavlink — przewodnik](https://mavlink.io/en/mavgen_python/)
- [kod pakietu](https://github.com/ArduPilot/pymavlink)
- [definicje common](https://mavlink.io/en/messages/common.html)
- [definicje ardupilotmega](https://mavlink.io/en/messages/ardupilotmega.html)
- [GUIDED ArduCopter](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)
- [symulacja Mission Planner](https://ardupilot.org/planner/docs/mission-planner-simulation.html)

Po instalacji zanotuj `python -m pip show pymavlink`, wersję Pythona i firmware. Sprawdzaj dokładne sygnatury wybranej wersji przez `inspect.signature(conn.mav.command_long_send)` i analogicznie dla pozostałych metod. Nie instaluj dodatkowego pakietu o nazwie ardupilotmega: dialekt dostarcza pymavlink.
