"""Logika misji "Kukurin nad Miasteczkiem".

Przebieg:
  1. przygotowanie (Vehicle.prepare_for_start + test głośnika),
  2. start na TAKEOFF_ALTITUDE_M,
  3. koło pomiarowe o promieniu CIRCLE_RADIUS_M -> średni pobór prądu,
  4. calculate_maximum_time() -> maksymalny czas i zasięg przy CRUISE_SPEED_MPS,
  5. plan: czy dolot + minuta muzyki na spirali + powrót mieszczą się w baterii (z 10% zapasu),
  6. dolot do celu (strażnik energii i ETA działa cały czas),
  7. muzyka + spirala (-2)^x z prędkością SPIRAL_SPEED_MPS, walidacja głośnika po kilku sekundach,
  8. sukces, jeśli spirala ma co najmniej SPIRAL_MIN_DIAMETER_M średnicy,
  9. powrót nad Home, pionowe lądowanie, potwierdzenie rozbrojenia.

Reakcje na nietypowe sytuacje są w KukurinMission.run() (patrz _handle_inflight_failure).
"""
import logging
import math as m
import time
from dataclasses import dataclass, field

from conf import (
    TAKEOFF_ALTITUDE_M, CRUISE_SPEED_MPS, TARGET_REACHED_TOLERANCE_M, HOME_REACHED_TOLERANCE_M,
    LAND_DESCENT_SPEED_MPS, CIRCLE_RADIUS_M, CIRCLE_SPEED_MPS, CIRCLE_LAPS, MIN_CURRENT_SAMPLES,
    BATTERY_RESERVE_FRACTION, ENERGY_SAFETY_FACTOR, MUSIC_DURATION_S, SPEAKER_CURRENT_A,
    SPEAKER_VALIDATION_S, MUSIC_RESTART_ATTEMPTS, SPIRAL_SPEED_MPS, SPIRAL_START_RADIUS_M,
    SPIRAL_MIN_DIAMETER_M, ETA_LOG_INTERVAL_S,
)
from errors import (
    MissionNotFeasible, EnergyBudgetExceeded, LowBatteryInFlight, LinkLostError,
    PilotOverrideError, MusicPlaybackError, CommandError,
)
from functions import (
    offset_point, circle_offset, spiral_offset, spiral_diameter, horizontal_distance,
    usable_mah, calculate_maximum_time, max_reachable_distance, return_home_mah,
    seconds_for, plan_mission,
)

log = logging.getLogger("drone.mission")


# =============================================================================
# Strażnik energii i ETA
# =============================================================================
class EnergyGuard:
    """Wołany po każdym Vehicle.poll(). Liczy ETA i pilnuje, żeby zawsze starczyło na powrót.

    Rzuca EnergyBudgetExceeded, gdy zostało już tylko tyle energii, ile potrzeba
    na powrót nad Home i lądowanie (z zapasem BATTERY_RESERVE_FRACTION i ENERGY_SAFETY_FACTOR).
    """

    def __init__(self, fly_current_a, speaker_current_a):
        self.fly_current_a = fly_current_a
        self.speaker_current_a = speaker_current_a
        self.live_current_a = fly_current_a     # średnia krocząca bieżącego poboru
        self.speaker_on = False
        self.virtual_speaker_mah = 0.0          # pobór głośnika, którego bateria SITL nie widzi
        self.enabled = True                     # False = tylko licz i loguj (np. w drodze powrotnej)
        self.eta_target = None                  # punkt, do którego liczymy ETA (oprócz Home)
        self._last_update = None
        self._last_eta_log = 0.0

    # --- liczenie
    def return_current_a(self):
        """Prąd do szacowania powrotu: ostrożniej z pomiaru z koła i bieżącej średniej."""
        return max(self.fly_current_a, self.live_current_a)

    def usable_mah(self, vehicle):
        return usable_mah(vehicle.battery_capacity_mah, vehicle.battery_status,
                          BATTERY_RESERVE_FRACTION, vehicle.consumed_mah, self.virtual_speaker_mah)

    def return_home_mah(self, vehicle):
        if vehicle.position is None or vehicle.home is None:
            return 0.0
        return return_home_mah(horizontal_distance(vehicle.position, vehicle.home),
                               vehicle.position[2], CRUISE_SPEED_MPS, LAND_DESCENT_SPEED_MPS,
                               self.return_current_a(), ENERGY_SAFETY_FACTOR)

    def spare_mah(self, vehicle):
        """Ładunek ponad to, co jest potrzebne na powrót i zapas."""
        return self.usable_mah(vehicle) - self.return_home_mah(vehicle)

    # --- wywołanie z poll()
    def __call__(self, vehicle):
        now = time.monotonic()
        if self._last_update is not None and self.speaker_on:
            self.virtual_speaker_mah += self.speaker_current_a * (now - self._last_update) / 3.6
        self._last_update = now
        if vehicle.current_a is not None:
            self.live_current_a = 0.9 * self.live_current_a + 0.1 * vehicle.current_a

        if vehicle.battery_status is None or vehicle.battery_status < 0 or vehicle.position is None:
            return
        spare = self.spare_mah(vehicle)

        if now - self._last_eta_log >= ETA_LOG_INTERVAL_S:
            self._last_eta_log = now
            self.log_eta(vehicle, spare)

        if self.enabled and spare <= 0:
            raise EnergyBudgetExceeded(
                f"Zostało tylko {self.usable_mah(vehicle):.0f} mAh ponad zapas, a powrót wymaga "
                f"{self.return_home_mah(vehicle):.0f} mAh - wracam"
            )

    def log_eta(self, vehicle, spare):
        current = self.return_current_a() + (self.speaker_current_a if self.speaker_on else 0)
        parts = []
        if self.eta_target is not None:
            d = horizontal_distance(vehicle.position, self.eta_target)
            parts.append(f"ETA cel: {d / CRUISE_SPEED_MPS:.0f} s ({d:.0f} m)")
        d_home = horizontal_distance(vehicle.position, vehicle.home)
        parts.append(f"ETA Home: {d_home / CRUISE_SPEED_MPS:.0f} s ({d_home:.0f} m)")
        parts.append(f"zapas ponad powrót: {spare:.0f} mAh (~{seconds_for(max(spare, 0), current):.0f} s)")
        if self.virtual_speaker_mah > 0:
            parts.append(f"głośnik: {self.virtual_speaker_mah:.1f} mAh")
        log.info(" | ".join(parts))


# =============================================================================
# Wynik misji
# =============================================================================
@dataclass
class MissionResult:
    success: bool = False
    reason: str = ""
    landed_home: bool = False
    data: dict = field(default_factory=dict)


# =============================================================================
# Misja
# =============================================================================
class KukurinMission:
    def __init__(self, vehicle, speaker, target, auto_confirm=False):
        self.vehicle = vehicle
        self.speaker = speaker
        self.target = (target[0], target[1], TAKEOFF_ALTITUDE_M)
        self.auto_confirm = auto_confirm
        self.guard = None
        self.result = MissionResult()

        # stan spirali
        self._spiral_duration = 0.0
        self._spiral_max_radius = 0.0
        self._spiral_samples_start = 0
        self._speaker_validated = False
        self._music_restarts = 0
        self._baseline_current = None
        self._warned_small_spiral = False

    # ------------------------------------------------------------------ główna
    def run(self):
        v = self.vehicle
        # 1. przygotowanie - dron na ziemi, błędy lecą prosto do main.py
        self.speaker.prepare()
        v.prepare_for_start(confirm=not self.auto_confirm)
        log.info("Cel: %.6f, %.6f (%.0f m od Home)", self.target[0], self.target[1],
                 horizontal_distance(v.home, self.target))

        # 2. start
        v.start_flight()
        self._enable_flight_watches()
        v.set_speed(CRUISE_SPEED_MPS)

        try:
            # 3-5. koło, maksymalny czas, plan
            fly_current = self.measure_current_on_circle()
            self.calculate_maximum_time(fly_current)
            plan = self.check_feasibility(fly_current)

            # 6. dolot
            self.guard = EnergyGuard(fly_current, SPEAKER_CURRENT_A)
            self.guard.eta_target = self.target
            v.monitors.append(self.guard)
            self.fly_to_target(plan)

            # 7-8. muzyka i spirala
            self.play_music_on_spiral(plan)
        except MissionNotFeasible as e:
            log.warning("Misja niewykonalna: %s - wracam bez lotu do celu", e)
            self.result.reason = f"niewykonalna: {e}"
        except (EnergyBudgetExceeded, LowBatteryInFlight, MusicPlaybackError,
                TimeoutError, CommandError) as e:
            self._handle_inflight_failure(e)
        except (LinkLostError, PilotOverrideError):
            self.speaker.stop()
            raise            # tu nie da się bezpiecznie dalej sterować - decyduje main.py

        # 9. powrót i lądowanie (także po przerwaniu misji)
        self.return_and_land()
        return self.result

    # ------------------------------------------------------------------ fazy
    def _enable_flight_watches(self):
        v = self.vehicle
        v.last_position_time = time.monotonic()
        v.link_watch = True
        v.battery_floor_watch = True

    def measure_current_on_circle(self):
        """Zatacza koło o promieniu CIRCLE_RADIUS_M i zwraca średni pobór prądu [A]."""
        v = self.vehicle
        center = v.position
        alt = TAKEOFF_ALTITUDE_M
        start_n, start_e = circle_offset(0, CIRCLE_RADIUS_M, CIRCLE_SPEED_MPS)
        start = (*offset_point(center[0], center[1], start_n, start_e), alt)
        log.info("Koło pomiarowe: promień %.1f m, %.1f m/s", CIRCLE_RADIUS_M, CIRCLE_SPEED_MPS)
        v.goto_position(start)
        v.wait_until_reached(start, TARGET_REACHED_TOLERANCE_M, timeout=30)

        duration = CIRCLE_LAPS * 2 * m.pi * CIRCLE_RADIUS_M / CIRCLE_SPEED_MPS

        def point(t):
            north, east = circle_offset(t, CIRCLE_RADIUS_M, CIRCLE_SPEED_MPS)
            return offset_point(center[0], center[1], north, east)

        v.start_current_recording()
        v.follow_trajectory(point, duration, alt)
        samples = v.stop_current_recording()

        if len(samples) < MIN_CURRENT_SAMPLES:
            if v.current_a is None:
                raise MissionNotFeasible("brak pomiarów prądu - nie da się policzyć budżetu energii")
            log.warning("Tylko %d pomiarów prądu - przyjmuję bieżący odczyt %.1f A",
                        len(samples), v.current_a)
            avg = v.current_a
        else:
            avg = sum(samples) / len(samples)
        log.info("Średni pobór prądu na kole: %.2f A (%d pomiarów, min %.1f, maks %.1f)",
                 avg, len(samples), min(samples, default=avg), max(samples, default=avg))
        self.result.data['avg_current_a'] = round(avg, 2)

        v.goto_position(center[:2] + (alt,))      # wracamy nad środek koła
        return avg

    def calculate_maximum_time(self, avg_current_a):
        """Maksymalny czas lotu i zasięg przy CRUISE_SPEED_MPS, z 10% bezwzględnego zapasu."""
        v = self.vehicle
        max_time = calculate_maximum_time(v.battery_capacity_mah, v.battery_status, avg_current_a,
                                          BATTERY_RESERVE_FRACTION, v.consumed_mah)
        total, radius = max_reachable_distance(max_time, CRUISE_SPEED_MPS)
        log.info("Maksymalny czas lotu: %.0f s (%.1f min) | przy %.0f m/s: trasa %.0f m, "
                 "zasięg tam i z powrotem %.0f m",
                 max_time, max_time / 60, CRUISE_SPEED_MPS, total, radius)
        self.result.data.update(max_time_s=round(max_time), max_route_m=round(total),
                                max_radius_m=round(radius))
        return max_time, total, radius

    def check_feasibility(self, fly_current_a):
        """Czy dolot, minuta Kukurina na full (spirala) i powrót są wykonalne."""
        v = self.vehicle
        usable = usable_mah(v.battery_capacity_mah, v.battery_status, BATTERY_RESERVE_FRACTION,
                            v.consumed_mah)
        plan = plan_mission(
            usable=usable,
            fly_current_a=fly_current_a,
            speaker_current_a=SPEAKER_CURRENT_A,
            distance_out_m=horizontal_distance(v.position, self.target),
            distance_back_m=horizontal_distance(self.target, v.home),
            altitude_m=TAKEOFF_ALTITUDE_M,
            cruise_speed=CRUISE_SPEED_MPS,
            land_speed=LAND_DESCENT_SPEED_MPS,
            music_duration_s=MUSIC_DURATION_S,
            spiral_speed=SPIRAL_SPEED_MPS,
            spiral_r0=SPIRAL_START_RADIUS_M,
            spiral_min_diameter=SPIRAL_MIN_DIAMETER_M,
            safety_factor=ENERGY_SAFETY_FACTOR,
        )
        log.info("Plan: %s", plan.summary())
        self.result.data.update(planned_spiral_diameter_m=round(plan.spiral_diameter_m, 2),
                                max_spiral_diameter_m=round(plan.max_spiral_diameter_m, 2))
        if not plan.can_reach_and_return:
            raise MissionNotFeasible("nie starczy baterii nawet na dolot i powrót")
        if not plan.success_possible:
            raise MissionNotFeasible(
                f"spirala wyniosłaby tylko {plan.spiral_diameter_m:.2f} m średnicy "
                f"(wymagane {SPIRAL_MIN_DIAMETER_M} m)")
        if not plan.full_music_possible:
            log.warning("Minuta muzyki się nie zmieści - spirala potrwa %.0f s", plan.spiral_time_s)
        return plan

    def fly_to_target(self, plan):
        v = self.vehicle
        log.info("Lot do celu: %.0f m, ETA %.0f s", plan.distance_out_m, plan.out_time_s)
        v.goto_position(self.target)
        v.wait_until_reached(self.target, TARGET_REACHED_TOLERANCE_M,
                             timeout=plan.out_time_s * 2 + 30)
        log.info("Cel osiągnięty - stabilizacja i pomiar prądu w zawisie")
        v.start_current_recording()
        v.wait_until_stable()
        samples = v.stop_current_recording()
        self._baseline_current = (sum(samples) / len(samples)) if samples else self.guard.fly_current_a
        log.info("Prąd w zawisie nad celem: %.2f A", self._baseline_current)

    def play_music_on_spiral(self, plan):
        v = self.vehicle
        center = self.target
        self._spiral_duration = plan.spiral_time_s
        self._spiral_max_radius = 0.0
        log.info("Kukurin na full + spirala (-2)^x: %.0f s, %.2f m/s, planowana średnica %.2f m",
                 plan.spiral_time_s, SPIRAL_SPEED_MPS, plan.spiral_diameter_m)

        def point(t):
            north, east = spiral_offset(SPIRAL_SPEED_MPS * t, SPIRAL_START_RADIUS_M)
            return offset_point(center[0], center[1], north, east)

        v.start_current_recording()
        self.speaker.play()
        self.guard.speaker_on = True
        started = time.monotonic()
        try:
            v.follow_trajectory(point, lambda: self._spiral_duration, center[2],
                                on_tick=self._spiral_tick)
        finally:
            # zapisujemy wynik spirali także wtedy, gdy została przerwana
            flown = time.monotonic() - started
            self.guard.speaker_on = False
            self.speaker.stop()
            v.stop_current_recording()
            flown_diameter = 2 * self._spiral_max_radius
            planned_diameter = spiral_diameter(SPIRAL_SPEED_MPS * flown, SPIRAL_START_RADIUS_M)
            self.result.data.update(spiral_time_s=round(flown, 1),
                                    spiral_diameter_planned_m=round(planned_diameter, 2),
                                    spiral_diameter_flown_m=round(flown_diameter, 2),
                                    speaker_virtual_mah=round(self.guard.virtual_speaker_mah, 1))
        log.info("Spirala zakończona: %.0f s, średnica wg GPS %.2f m (teoretycznie %.2f m)",
                 flown, flown_diameter, planned_diameter)
        if flown_diameter >= SPIRAL_MIN_DIAMETER_M:
            self.result.success = True
            self.result.reason = f"spirala {flown_diameter:.2f} m >= {SPIRAL_MIN_DIAMETER_M} m"
            log.info("MISJA WYKONANA: %s", self.result.reason)
        else:
            self.result.reason = f"spirala za mała: {flown_diameter:.2f} m < {SPIRAL_MIN_DIAMETER_M} m"
            log.warning("Misja niewykonana: %s", self.result.reason)

    def _spiral_tick(self, t):
        """Wołane w każdej iteracji spirali: głośnik, walidacja poboru, budżet energii."""
        v = self.vehicle
        if v.position is not None:
            self._spiral_max_radius = max(self._spiral_max_radius,
                                          horizontal_distance(v.position, self.target))

        # 1. czy muzyka gra
        if not self.speaker.is_playing():
            if self._music_restarts < MUSIC_RESTART_ATTEMPTS:
                self._music_restarts += 1
                log.warning("Muzyka nie gra - ponowne uruchomienie (%d/%d)",
                            self._music_restarts, MUSIC_RESTART_ATTEMPTS)
                self.speaker.stop()
                self.speaker.play()
            else:
                raise MusicPlaybackError(f"Muzyka przestała grać po {t:.0f} s spirali")

        # 2. walidacja po kilku sekundach: odtwarzanie i pobór prądu głośnika
        if not self._speaker_validated and t >= SPEAKER_VALIDATION_S:
            self._speaker_validated = True
            self._validate_speaker(t)

        # 3. ile spirali jeszcze się zmieści, żeby starczyło na powrót
        # margines 2% pojemności, żeby spirala skończyła się sama, zanim zadziała strażnik energii
        spare = self.guard.spare_mah(v) - 0.02 * v.battery_capacity_mah
        current = self.guard.return_current_a() + self.guard.speaker_current_a
        allowed = t + seconds_for(max(spare, 0.0), current)
        new_duration = min(MUSIC_DURATION_S, allowed)
        if new_duration < self._spiral_duration - 0.5:
            self._spiral_duration = new_duration
            final_d = spiral_diameter(SPIRAL_SPEED_MPS * new_duration, SPIRAL_START_RADIUS_M)
            if final_d < SPIRAL_MIN_DIAMETER_M and not self._warned_small_spiral:
                self._warned_small_spiral = True
                log.warning("Bateria pozwala tylko na %.0f s spirali (średnica %.2f m < %.1f m)",
                            new_duration, final_d, SPIRAL_MIN_DIAMETER_M)

    def _validate_speaker(self, t):
        v = self.vehicle
        played = self.speaker.elapsed_s()
        if played < 0.5 * SPEAKER_VALIDATION_S:
            raise MusicPlaybackError(
                f"Odtwarzanie nie postępuje: zagrało {played:.1f} s z {t:.1f} s")

        samples = v._current_samples or []
        if samples and self._baseline_current is not None:
            measured = sum(samples) / len(samples) - self._baseline_current
            log.info("Walidacja głośnika: pobór z pomiaru %.2f A, z konfiguracji %.2f A",
                     measured, SPEAKER_CURRENT_A)
            if measured > SPEAKER_CURRENT_A * 1.2:
                log.warning("Głośnik pobiera więcej niż w conf - przeliczam budżet z %.2f A", measured)
                self.guard.speaker_current_a = measured
            elif measured < 0.5 * SPEAKER_CURRENT_A:
                log.info("Pomiar nie potwierdza poboru z conf (w SITL głośnik nie obciąża baterii) "
                         "- zostaję przy wartości z conf")
            self.result.data['speaker_measured_a'] = round(measured, 2)
        log.info("Głośnik gra poprawnie (%.1f s utworu)", played)

    # ------------------------------------------------------------------ awarie i powrót
    def _handle_inflight_failure(self, error):
        """Problem w locie, ale nadal mamy kontrolę nad dronem: przerywamy i wracamy sami."""
        log.error("Przerwanie misji: %s: %s", type(error).__name__, error)
        self.speaker.stop()
        if self.guard is not None:
            self.guard.speaker_on = False
        self.result.success = False
        self.result.reason = f"przerwana: {type(error).__name__}: {error}"

    def return_and_land(self):
        """Powrót nad Home i lądowanie. Jeśli to się nie uda - RTL/LAND autopilota."""
        v = self.vehicle
        if self.guard is not None:
            self.guard.enabled = False        # w drodze powrotnej strażnik tylko loguje
            self.guard.eta_target = None
        v.battery_floor_watch = False         # nie przerywamy powrotu z powodu baterii
        try:
            v.set_speed(CRUISE_SPEED_MPS)
            v.return_home(TAKEOFF_ALTITUDE_M, HOME_REACHED_TOLERANCE_M)
            v.land()
            v.check_disarm_confirmation()
            self.result.landed_home = True
        except (LinkLostError, PilotOverrideError):
            raise
        except Exception as e:
            log.error("Własny powrót nie powiódł się (%s: %s) - przekazuję autopilotowi",
                      type(e).__name__, e)
            v.emergency_return()
            self.result.reason += " | powrót przez RTL/LAND autopilota"
