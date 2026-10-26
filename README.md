# Misja: Kukurin nad Miasteczkiem Studenckim AGH

Rozwiązanie zadania rekrutacyjnego AGH Solar Plane 2026 (treść: `Zadanie_rekrutacyjne (1).md`).

## Pełne nagranie symulacji misji w misja_kukurin_90s.mp4

## 1. Koncepcja misji
Dron (ArduCopter) startuje, sprawdza, ile realnie zużywa prądu, i ocenia, czy starczy mu baterii na misję.
Jeśli tak, leci nad Miasteczko Studenckie AGH, odtwarza na pełnej głośności minutę nagrania `Kukurin.mp3`
i w tym czasie zatacza powiększającą się spiralę. Potem wraca nad punkt startu i ląduje.

Czujnikiem w misji jest pomiar prądu baterii: na jego podstawie dron decyduje, czy w ogóle leci do celu
i jak długo może grać.

## 2. Przebieg prawidłowego scenariusza
1. Kontrole przed startem: łączność, GPS, EKF, punkt Home, bateria (min. 50%), gotowość głośnika.
2. Start na 12 m (tryb GUIDED).
3. Koło o promieniu 5 m: pomiar średniego poboru prądu.
4. Obliczenie maksymalnego czasu lotu i zasięgu (z bezwzględnym zapasem 10% baterii).
5. Plan: czy dolot, minuta muzyki i powrót zmieszczą się w baterii.
6. Lot do celu z prędkością 10 m/s, z bieżącym liczeniem ETA i zapasu energii.
7. Muzyka i spirala (-2)^x z prędkością 20 cm/s, sprawdzenie głośnika po 5 s.
8. Powrót nad Home, pionowe lądowanie, automatyczne rozbrojenie.

**Sukces misji:** spirala ma co najmniej 5 m średnicy (według GPS), co przy 20 cm/s odpowiada ok. minucie grania,
a dron wylądował w Home. Program kończy się wtedy kodem `0`.

## 3. Architektura
| Plik | Odpowiedzialność |
|---|---|
| `main.py` | połączenie z autopilotem, uruchomienie misji, ostatnia linia obsługi błędów (RTL, Ctrl+C), kody wyjścia |
| `mission.py` | przebieg misji, plan energetyczny, strażnik energii, reakcje na awarie |
| `vehicle.py` | komunikacja MAVLink: telemetria, tryby, uzbrojenie, start, lot do punktu, lądowanie |
| `speaker.py` | głośnik: odtwarzanie przez pygame albo tryb symulowany |
| `functions.py` | czyste obliczenia: geometria, spirala, energia |
| `errors.py` | wyjątki (osobno: przed startem / w locie / głośnik) |
| `conf.py` | wszystkie ustawienia i progi |

Zasada: `vehicle.py` tylko wykrywa i zgłasza problemy (wyjątki), a decyzję o reakcji podejmuje `mission.py` i `main.py`.

## 4. Uruchomienie
```
pip install -r requirements.txt
python main.py
```
Przed startem program pyta o zgodę - należy odpowiedzieć `TAK`.

Opcje: `--yes` (bez pytań), `--speaker simulated` (bez dźwięku), `--fault-music 20` (test awarii muzyki),
`--target LAT LON` (inny cel). Pełny log każdego lotu trafia do katalogu `logs/`.

Kody wyjścia: `0` sukces, `1` misja niewykonana / przerwana, `2` przerwanie przez operatora.

## 5. Symulacja w Mission Plannerze (SITL)
1. Zakładka **SIMULATION**, w polu **Extra command line** wpisz:
   `--home=50.0670,19.9100,205,0` (ok. 350 m od celu).
2. Wybierz **Multirotor** i poczekaj, aż dron pojawi się na mapie.
3. Uruchom `python main.py`. Program łączy się z drugim portem symulatora: `tcp:127.0.0.1:5762`.

Port obsługuje jedno połączenie naraz, więc poprzednie uruchomienie programu musi być zakończone.

## 6. Testy
```
python -m pytest -q
```
33 testy, bez SITL: geometria, spirala (zgodność z (-2)^x, długość łuku), obliczenia energii,
plan misji, strażnik energii, reakcja na awarię muzyki.

## 7. Scenariusze testowe (sprawdzone w SITL)
| Scenariusz | Jak wywołać | Oczekiwany wynik | Kod |
|---|---|---|---|
| prawidłowy przebieg | home 350 m od celu | minuta muzyki, spirala 5.29 m, powrót i lądowanie | `0` |
| cel za daleko | home na Błoniach (1.24 km) | plan: spirala tylko 4.04 m, powrót bez lotu do celu | `1` |
| muzyka przestaje grać | `--fault-music 20` | jedna próba ponownego uruchomienia, potem powrót | `1` |
| za mało baterii w locie | zmniejszenie `BATT_CAPACITY` w trakcie lotu | spirala skrócona albo przerwanie i powrót | `1` |
| przejęcie drona | zmiana trybu w Mission Plannerze | program przestaje wysyłać polecenia | `1` |
| utrata łączności | zamknięcie symulatora w trakcie lotu | 3 próby odzyskania, próba RTL, dalej działa failsafe autopilota | `1` |
| Ctrl+C w locie | Ctrl+C w terminalu | przełączenie w RTL | `2` |
| brak pliku z muzyką | usunięcie `Kukurin.mp3` | start wstrzymany na ziemi | `1` |

### Jak liczony jest budżet energii
- maksymalny czas lotu = (bateria − 10% zapasu) / średni prąd z koła,
- w locie stale: zapas = bateria do użycia − koszt powrotu (× 1.2); gdy zapas spadnie do zera, dron wraca,
- spirala jest skracana na bieżąco, tak żeby zawsze starczyło na powrót.

## 8. Uproszczenia i ograniczenia symulacji
- Głośnik: w SITL go nie ma. Muzyka gra z komputera, a pobór prądu głośnika (2 A, wartość przyjęta)
  jest doliczany wirtualnie, bo bateria SITL go nie widzi.
- Bateria: model SITL ma prawie stały pobór i nie uwzględnia spadku napięcia ani temperatury.
- Brak wiatru i przeszkód: plan zakłada lot w linii prostej, a wysokość jest liczona względem Home, nie terenu.
- Prąd mierzony przy 2 m/s, a przelot jest z 10 m/s. Przy powrocie bierzemy większą z wartości: średnią z koła albo bieżącą.
- Decyzja o wykonalności zapada w powietrzu, bo dopiero po kole znamy realny pobór.
- Utrata łączności: program nie może już nic zrobić, bezpieczeństwo zapewnia failsafe autopilota.
- Mały margines sukcesu: spirala ma teoretycznie 5.37 m, w SITL 5.29 m, przy wymaganych 5.0 m.
- Scenariusze testowano na ArduCopter 4.5; Mission Planner w tym README używa wersji 4.8-dev.

## 9. Pomysły na rozwój
- sprawdzanie parametrów failsafe autopilota przed startem,
- wstępne szacowanie energii jeszcze przed startem,
- prawdziwy pomiar poboru głośnika i potwierdzenie dźwięku mikrofonem,
- uwzględnienie wiatru w planie energetycznym,
- cel wyznaczany przez detekcję z kamery zamiast stałych współrzędnych.

## 10. Wykorzystane źródła i narzędzia
- [pymavlink](https://github.com/ArduPilot/pymavlink), [MAVLink – common messages](https://mavlink.io/en/messages/common.html)
- [ArduPilot – Copter Commands in Guided Mode](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)
- [Mission Planner Simulation](https://ardupilot.org/planner/docs/mission-planner-simulation.html)
- [Mureka - do stworzenia pliku dźwiękowego wolnego od praw autorskich](https://www.mureka.ai/home)
- pygame (odtwarzanie dźwięku), pytest (testy)
- Przy pracy korzystałem z asystenta AI (Claude) - do wyjaśniania MAVLinka, przeglądu kodu i części implementacji. Praca ręczna zajęła mi około 15 godzin pracy - oprogramowanie pozwalało na start i lot drona w zadanym kierunku, a także powrót, bez lądowania. Reszta kodu (od momentu implementacji lądowania aż po koniec implementacji misji została napisana przez agenta). 
