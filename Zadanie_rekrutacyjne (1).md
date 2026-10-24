# Zadanie rekrutacyjne AGH Solar Plane 2026
Gratulacje! Udało Ci się przejść pierwszy etap rekrutacji do zespołu programowania.
Teraz chcemy zaprosić Cię do zmierzenia się z zadaniem zbliżonym do problemów, które rozwiązujemy podczas przygotowań do zawodów.
### Wprowadzenie
W AGH Solar Plane projektujemy samoloty oraz drony realizujące autonomiczne misje. W zależności od zadania nasze konstrukcje mogą być wyposażone między innymi w:
- komputer pokładowy, np. NVIDIA Jetson lub Raspberry Pi,
- kamerę RGB,
- kamerę termowizyjną,
- LiDAR,
- system zrzutu ładunków,

Sprzęt i oprogramowanie muszą współpracować tak, aby statek powietrzny był w stanie autonomicznie wykonać zaplanowaną misję.
Przykładowe zadania na zawodach obejmują:
- mapowanie wskazanego obszaru,
- wykrywanie i lokalizowanie obiektów,
- wykonywanie precyzyjnych zrzutów,
- planowanie trasy na podstawie danych z czujników.

Inspiracji możesz szukać w zadaniach z zawodów:
- [Student Unmanned Aerial Systems Competition – SUAS](https://www.auvsi-suas.org/),
- [International Micro Air Vehicle Conference and Competition – IMAV](https://www.imavcompetition.org/),
- [Droniada](https://droniada.eu/).

Dobre oprogramowanie powinno obsługiwać nie tylko oczekiwany przebieg misji, lecz także sytuacje nietypowe: brak detekcji, błędne dane, utratę komunikacji, przekroczenie limitu czasu czy niemożność wykonania zadania.
Zanim oprogramowanie trafi na prawdziwy statek powietrzny, staramy się przetestować jak największą część systemu bez wykonywania lotów. Pozwala to oszczędzić czas przeznaczony na testy terenowe i rozwijać projekt również wtedy, gdy docelowa maszyna nie jest dostępna.

### Cel zadania
Zaprojektuj i zaimplementuj oprogramowanie dla samolotu lub drona, które realizuje wymyśloną przez Ciebie autonomiczną misję.
Następnie przygotuj sposób przetestowania rozwiązania z wykorzystaniem symulacji SITL (software in the loop) w Mission Plannerze.
Instrukcję uruchamiania symulacji bezpośrednio z Mission Plannera znajdziesz w [dokumentacji Mission Planner Simulation](https://ardupilot.org/planner/docs/mission-planner-simulation.html). Instalator programu jest dostępny na [oficjalnej stronie pobierania](https://ardupilot.org/planner/docs/mission-planner-installation.html). 
Nie oczekujemy kompletnego systemu gotowego do lot u na zawodach. Chcemy zobaczyć, w jaki sposób projektujesz rozwiązanie oraz testujesz jego zachowanie.

### Wymagania misji
Misja powinna zawierać co najmniej:
1. Start statku powietrznego.
2. Przelot nad wybranym obszarem lub wzdłuż zaplanowanej trasy.
3. Zadanie wymagające wykorzystania kamery, LiDAR-u albo innego wybranego czujnika.
4. Reakcję systemu na wynik działania czujnika lub algorytmu.
5. Powrót i lądowanie.

Nie musisz implementować zaawansowanego modelu uczenia maszynowego. Możesz wykorzystać prosty algorytm, gotowy model, dane testowe albo zasymulowane wyniki detekcji. Istotniejsze jest poprawne zaprojektowanie całego przepływu misji i reakcji systemu.

### Testowanie
Zapoznaj się z podstawowymi funkcjami Mission Plannera, a następnie skonfiguruj symulację pozwalającą sprawdzić działanie Twojego programu.
Nie wszystkie elementy rzeczywistego systemu da się zasymulować bezpośrednio w Mission Plannerze. Twoim zadaniem jest zdecydowanie, które części można przetestować w SITL, a dla których należy przygotować inne źródło danych.

### Co powinno znaleźć się w repozytorium?
Rozwiązanie należy umieścić w publicznym repozytorium na GitHubie i przesłać nam link.
Repozytorium powinno zawierać:
- kod źródłowy,
- pliki potrzebne do uruchomienia misji,
- wykorzystane lub przykładowe dane testowe,
- automatyczne testy, jeżeli zostały przygotowane,
- plik README.md.

W README.md opisz:
1. Koncepcję misji.
2. Przebieg prawidłowego scenariusza.
3. Architekturę rozwiązania i najważniejsze komponenty.
5. Dokładną instrukcję uruchomienia programu.
6. Sposób uruchomienia symulacji w Mission Plannerze/SITL.
7. Sposób uruchomienia testów.
8. Przygotowane scenariusze testowe i ich oczekiwane wyniki.
9. Przyjęte uproszczenia i elementy, których nie udało się zasymulować.
10. Pomysły na dalszy rozwój rozwiązania.

Instrukcja powinna pozwolić innej osobie uruchomić projekt bez konieczności domyślania się brakujących kroków.

Zasady
- Możesz korzystać z dowolnych materiałów, bibliotek, gotowych modeli oraz narzędzi AI.
- Wyraźnie zaznacz wykorzystane zewnętrzne źródła i komponenty.
- Możesz wybrać dowolny język programowania.
- Możesz skorzystać z gotowych danych albo przygotować własne.
- Nie musisz posiadać fizycznego drona, kontrolera lotu ani kamery.
- Nie oczekujemy interfejsu graficznego, chyba że jest istotną częścią Twojego pomysłu.
- Ważniejsze od liczby funkcji są: poprawność działania, czytelność kodu, sensowna architektura i jakość testów.

Kryteria oceny
Podczas oceny zwrócimy uwagę na:
- pomysł i sensowność zaproponowanej misji,
- sposób podziału systemu na komponenty,
- poprawność komunikacji z symulowanym autopilotem,
- jakość logiki sterującej przebiegiem misji,
- obsługę błędów i nieoczekiwanych sytuacji,
- jakość oraz powtarzalność testów,
- czytelność kodu,
- jakość dokumentacji,
- świadomość ograniczeń przygotowanej symulacji.

Nie musisz znać wcześniej Mission Plannera, ArduPilota ani MAVLinka. Częścią zadania jest samodzielne zapoznanie się z potrzebnymi narzędziami i udokumentowanie zdobytej wiedzy.
Powodzenia — liczymy na kreatywne pomysły i rozsądne podejście inżynierskie!