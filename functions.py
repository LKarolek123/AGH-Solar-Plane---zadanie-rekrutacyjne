"""Czyste funkcje pomocnicze: geometria, spirala, energia.

Nie korzystają z połączenia z dronem, więc łatwo je testować (tests/test_functions.py).
"""
import cmath
import math as m
from dataclasses import dataclass

from conf import EARTH_RADIUS_M


# =============================================================================
# Geometria na kuli (przybliżenie równoprostokątne - wystarczające do kilkudziesięciu km)
# =============================================================================

def destination_point(lat, lon, bearing_deg, distance_m):
    """Liczy współrzędne punktu oddalonego o distance_m w kierunku bearing_deg
    (0° = północ, 90° = wschód)."""
    d_east = distance_m * m.sin(m.radians(bearing_deg))
    d_north = distance_m * m.cos(m.radians(bearing_deg))
    return offset_point(lat, lon, d_north, d_east)


def offset_point(lat, lon, north_m, east_m):
    """Punkt przesunięty o north_m metrów na północ i east_m metrów na wschód."""
    d_lat = m.degrees(north_m / EARTH_RADIUS_M)
    d_lon = m.degrees(east_m / (EARTH_RADIUS_M * m.cos(m.radians(lat))))
    return lat + d_lat, lon + d_lon


def ne_offset(a, b):
    """Przesunięcie (north_m, east_m) od punktu a do punktu b (krotki zaczynające się od lat, lon)."""
    mean_lat = m.radians((a[0] + b[0]) / 2)
    north = m.radians(b[0] - a[0]) * EARTH_RADIUS_M
    east = m.radians(b[1] - a[1]) * m.cos(mean_lat) * EARTH_RADIUS_M
    return north, east


def horizontal_distance(a, b):
    """Odległość pozioma w metrach (bierze pod uwagę tylko lat, lon)."""
    north, east = ne_offset(a, b)
    return m.hypot(north, east)


def calculate_distance(start, current):
    """Odległość 3D w metrach między punktami (lat, lon, alt).
    Wysokości muszą być w tym samym układzie (u nas: względem Home)."""
    north, east = ne_offset(start, current)
    dz = current[2] - start[2]
    return round(m.sqrt(north * north + east * east + dz * dz), 3)


def circle_offset(t, radius_m, speed_mps):
    """Przesunięcie (north, east) po czasie t na okręgu o środku (0, 0).
    Start na północ od środka, ruch zgodnie z ruchem wskazówek zegara."""
    angle = speed_mps / radius_m * t
    return radius_m * m.cos(angle), radius_m * m.sin(angle)


# =============================================================================
# Spirala: rzut f(x) = (-2)^x na płaszczyznę zespoloną
# =============================================================================
#
# (-2)^x = 2^x * e^{i*pi*x}  (gałąź główna), czyli moduł r = 2^x, argument theta = pi*x.
# To spirala logarytmiczna r = e^{B*theta} z B = ln(2)/pi.
# Długość łuku spirali logarytmicznej od r0 do r1 ma postać zamkniętą:
#     L = sqrt(1 + B^2) / B * (r1 - r0)
# więc dla zadanej długości trasy od razu znamy promień, do którego dolecimy.

SPIRAL_B = m.log(2) / m.pi
SPIRAL_ARC_FACTOR = m.sqrt(1 + SPIRAL_B ** 2) / SPIRAL_B   # ≈ 4.64: metry trasy na 1 m przyrostu promienia


def spiral_length(r0, r1):
    """Długość trasy po spirali od promienia r0 do r1."""
    return SPIRAL_ARC_FACTOR * (r1 - r0)


def spiral_radius_after(length_m, r0):
    """Promień spirali po przebyciu length_m metrów (start z promienia r0)."""
    return r0 + max(length_m, 0.0) / SPIRAL_ARC_FACTOR


def spiral_diameter(length_m, r0):
    """Średnica spirali (2 * największy promień) po przebyciu length_m metrów."""
    return 2 * spiral_radius_after(length_m, r0)


def spiral_offset(length_m, r0):
    """Przesunięcie (north, east) punktu spirali po przebyciu length_m metrów.

    Punkt liczymy dosłownie jako (-2)^x: oś rzeczywista = wschód, urojona = północ.
    """
    r = spiral_radius_after(length_m, r0)
    x = m.log2(r)
    z = complex(-2, 0) ** x          # Python zwraca gałąź główną: 2^x * e^{i*pi*x}
    return z.imag, z.real


# =============================================================================
# Energia
# =============================================================================

def mah_for(duration_s, current_a):
    """Ładunek [mAh] zużyty przez duration_s sekund przy prądzie current_a [A]."""
    return current_a * duration_s / 3.6


def seconds_for(mah, current_a):
    """Ile sekund wystarczy mah [mAh] przy prądzie current_a [A]."""
    if current_a <= 0:
        return float('inf')
    return mah * 3.6 / current_a


def remaining_mah(capacity_mah, remaining_pct, consumed_mah=None):
    """Pozostały ładunek. Jeśli autopilot podaje zużycie, bierzemy ostrożniejszą z dwóch wartości."""
    from_pct = capacity_mah * max(remaining_pct, 0) / 100
    if consumed_mah is not None and consumed_mah >= 0:
        return min(from_pct, capacity_mah - consumed_mah)
    return from_pct


def usable_mah(capacity_mah, remaining_pct, reserve_fraction, consumed_mah=None, extra_used_mah=0.0):
    """Ładunek, który wolno zużyć, zostawiając bezwzględny zapas reserve_fraction * pojemność.

    extra_used_mah - zużycie, którego autopilot nie widzi (np. wirtualny głośnik w SITL).
    """
    left = remaining_mah(capacity_mah, remaining_pct, consumed_mah) - extra_used_mah
    return max(left - reserve_fraction * capacity_mah, 0.0)


def calculate_maximum_time(capacity_mah, remaining_pct, avg_current_a, reserve_fraction,
                           consumed_mah=None, extra_used_mah=0.0):
    """Maksymalny czas lotu [s] przy średnim prądzie avg_current_a, z zachowaniem zapasu."""
    usable = usable_mah(capacity_mah, remaining_pct, reserve_fraction, consumed_mah, extra_used_mah)
    return seconds_for(usable, avg_current_a)


def max_reachable_distance(max_time_s, speed_mps):
    """Zwraca (całkowita długość trasy, promień działania tam i z powrotem) w metrach."""
    total = max_time_s * speed_mps
    return total, total / 2


def return_home_mah(distance_home_m, altitude_m, cruise_speed, land_speed, current_a, safety_factor):
    """Szacowane zużycie na powrót nad Home i lądowanie (z mnożnikiem bezpieczeństwa)."""
    time_s = distance_home_m / cruise_speed + max(altitude_m, 0.0) / land_speed
    return mah_for(time_s, current_a) * safety_factor


@dataclass
class MissionPlan:
    """Wynik planowania misji: czasy, zużycie i rozmiar spirali."""
    usable_mah: float
    fly_current_a: float
    speaker_current_a: float
    distance_out_m: float
    distance_back_m: float
    out_time_s: float
    back_time_s: float
    out_mah: float
    back_mah: float
    spiral_budget_s: float        # ile czasu spirali mieści się w budżecie energii
    spiral_time_s: float          # zaplanowany czas spirali (budżet obcięty do długości muzyki)
    spiral_diameter_m: float      # średnica przy zaplanowanym czasie
    max_spiral_diameter_m: float  # średnica, gdyby latać spiralę aż do wyczerpania budżetu
    can_reach_and_return: bool
    full_music_possible: bool
    success_possible: bool

    def summary(self):
        return (
            f"budżet {self.usable_mah:.0f} mAh | dolot {self.distance_out_m:.0f} m "
            f"({self.out_time_s:.0f} s, {self.out_mah:.0f} mAh) | powrót+lądowanie "
            f"{self.distance_back_m:.0f} m ({self.back_time_s:.0f} s, {self.back_mah:.0f} mAh) | "
            f"spirala {self.spiral_time_s:.0f} s (budżet {self.spiral_budget_s:.0f} s) -> "
            f"średnica {self.spiral_diameter_m:.2f} m (maks. {self.max_spiral_diameter_m:.2f} m)"
        )


def plan_mission(usable, fly_current_a, speaker_current_a, distance_out_m, distance_back_m,
                 altitude_m, cruise_speed, land_speed, music_duration_s, spiral_speed,
                 spiral_r0, spiral_min_diameter, safety_factor):
    """Sprawdza, czy dolot, minuta muzyki na spirali i powrót mieszczą się w budżecie energii."""
    out_time = distance_out_m / cruise_speed
    back_time = distance_back_m / cruise_speed + max(altitude_m, 0.0) / land_speed
    out_mah = mah_for(out_time, fly_current_a)
    back_mah = mah_for(back_time, fly_current_a) * safety_factor

    spiral_current = fly_current_a + speaker_current_a
    left_for_spiral = usable - out_mah - back_mah
    spiral_budget = max(seconds_for(left_for_spiral, spiral_current), 0.0) if left_for_spiral > 0 else 0.0
    spiral_time = min(music_duration_s, spiral_budget)

    diameter = spiral_diameter(spiral_speed * spiral_time, spiral_r0)
    max_diameter = spiral_diameter(spiral_speed * spiral_budget, spiral_r0)

    return MissionPlan(
        usable_mah=usable,
        fly_current_a=fly_current_a,
        speaker_current_a=speaker_current_a,
        distance_out_m=distance_out_m,
        distance_back_m=distance_back_m,
        out_time_s=out_time,
        back_time_s=back_time,
        out_mah=out_mah,
        back_mah=back_mah,
        spiral_budget_s=spiral_budget,
        spiral_time_s=spiral_time,
        spiral_diameter_m=diameter,
        max_spiral_diameter_m=max_diameter,
        can_reach_and_return=usable >= out_mah + back_mah,
        full_music_possible=spiral_budget >= music_duration_s,
        success_possible=diameter >= spiral_min_diameter,
    )
