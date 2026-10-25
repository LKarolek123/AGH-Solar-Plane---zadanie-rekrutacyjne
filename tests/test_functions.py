"""Testy czystych funkcji: geometria, spirala, energia. Nie wymagają SITL.

Uruchomienie:  python -m pytest -q
"""
import cmath
import math as m

import pytest

from functions import (
    destination_point, offset_point, ne_offset, horizontal_distance, calculate_distance,
    circle_offset, SPIRAL_ARC_FACTOR, spiral_length, spiral_radius_after, spiral_diameter,
    spiral_offset, mah_for, seconds_for, remaining_mah, usable_mah, calculate_maximum_time,
    max_reachable_distance, return_home_mah, plan_mission,
)

KRAKOW = (50.0605, 19.9175)


# ----------------------------------------------------------------- geometria
@pytest.mark.parametrize("bearing", [0, 45, 90, 135, 180, 270, 333])
def test_destination_point_round_trip(bearing):
    lat, lon = destination_point(*KRAKOW, bearing, 100)
    assert calculate_distance((*KRAKOW, 0), (lat, lon, 0)) == pytest.approx(100, abs=0.1)


def test_offset_and_ne_offset_are_inverse():
    point = offset_point(*KRAKOW, 30, -40)
    north, east = ne_offset(KRAKOW, point)
    assert north == pytest.approx(30, abs=0.01)
    assert east == pytest.approx(-40, abs=0.01)


def test_horizontal_distance_is_in_meters_and_ignores_altitude():
    point = offset_point(*KRAKOW, 3, 4)
    assert horizontal_distance((*KRAKOW, 0), (*point, 50)) == pytest.approx(5, abs=0.01)


def test_calculate_distance_includes_altitude():
    point = offset_point(*KRAKOW, 3, 4)
    assert calculate_distance((*KRAKOW, 0), (*point, 12)) == pytest.approx(13, abs=0.01)


def test_circle_offset_keeps_radius_and_closes_after_one_lap():
    r, v = 5.0, 2.0
    lap = 2 * m.pi * r / v
    for t in (0, 1.3, lap / 2, lap * 0.77):
        assert m.hypot(*circle_offset(t, r, v)) == pytest.approx(r)
    assert circle_offset(lap, r, v) == pytest.approx(circle_offset(0, r, v), abs=1e-9)


# ------------------------------------------------------------------- spirala
def test_spiral_points_are_minus_two_to_the_x():
    for length in (0.0, 1.0, 5.0, 12.0):
        north, east = spiral_offset(length, 0.1)
        r = spiral_radius_after(length, 0.1)
        x = m.log2(r)
        expected = 2 ** x * cmath.exp(1j * m.pi * x)       # (-2)^x = 2^x e^{i pi x}
        assert east == pytest.approx(expected.real, abs=1e-9)
        assert north == pytest.approx(expected.imag, abs=1e-9)


def test_spiral_arc_length_matches_numeric_integration():
    """Długość z wzoru zgadza się z sumą odcinków łamanej po punktach spirali."""
    r0, length = 0.1, 12.0
    steps = 200_000
    pts = [spiral_offset(length * i / steps, r0) for i in range(steps + 1)]
    numeric = sum(m.dist(pts[i], pts[i + 1]) for i in range(steps))
    assert numeric == pytest.approx(length, rel=1e-3)
    assert spiral_length(r0, spiral_radius_after(length, r0)) == pytest.approx(length)


def test_spiral_arc_factor():
    assert SPIRAL_ARC_FACTOR == pytest.approx(4.642, abs=1e-3)


def test_one_minute_at_20cm_per_second_gives_at_least_5_m():
    assert spiral_diameter(0.2 * 60, 0.1) == pytest.approx(5.37, abs=0.01)
    assert spiral_diameter(0.2 * 60, 0.1) >= 5.0


def test_spiral_negative_length_is_clamped():
    assert spiral_radius_after(-3, 0.1) == pytest.approx(0.1)


# -------------------------------------------------------------------- energia
def test_mah_and_seconds_are_inverse():
    assert mah_for(3600, 1.0) == pytest.approx(1000)
    assert seconds_for(1000, 1.0) == pytest.approx(3600)
    assert seconds_for(100, 0) == float('inf')


def test_remaining_mah_takes_the_more_cautious_value():
    assert remaining_mah(5000, 80) == pytest.approx(4000)
    assert remaining_mah(5000, 80, consumed_mah=1500) == pytest.approx(3500)
    assert remaining_mah(5000, 80, consumed_mah=500) == pytest.approx(4000)


def test_usable_mah_keeps_absolute_10_percent_reserve():
    assert usable_mah(5000, 100, 0.10) == pytest.approx(4500)
    assert usable_mah(5000, 30, 0.10) == pytest.approx(1000)
    assert usable_mah(5000, 5, 0.10) == 0.0
    assert usable_mah(5000, 30, 0.10, extra_used_mah=200) == pytest.approx(800)


def test_calculate_maximum_time_and_range():
    # 5000 mAh, 100%, zapas 10% -> 4500 mAh; przy 30 A -> 540 s
    t = calculate_maximum_time(5000, 100, 30.0, 0.10)
    assert t == pytest.approx(540)
    total, radius = max_reachable_distance(t, 10.0)
    assert total == pytest.approx(5400)
    assert radius == pytest.approx(2700)


def test_return_home_mah():
    # 300 m przy 10 m/s = 30 s + 12 m / 0.5 m/s = 24 s -> 54 s przy 20 A = 300 mAh, x1.2
    assert return_home_mah(300, 12, 10, 0.5, 20, 1.2) == pytest.approx(360)


def _plan(usable, distance=300, speaker=2.0):
    return plan_mission(usable=usable, fly_current_a=20, speaker_current_a=speaker,
                        distance_out_m=distance, distance_back_m=distance, altitude_m=12,
                        cruise_speed=10, land_speed=0.5, music_duration_s=60,
                        spiral_speed=0.2, spiral_r0=0.1, spiral_min_diameter=5.0,
                        safety_factor=1.2)


def test_plan_feasible_with_full_minute():
    plan = _plan(3000)
    assert plan.can_reach_and_return
    assert plan.full_music_possible
    assert plan.spiral_time_s == pytest.approx(60)
    assert plan.success_possible
    assert plan.max_spiral_diameter_m > plan.spiral_diameter_m


def test_plan_battery_limits_spiral():
    # dolot 30 s * 20 A = 166.7 mAh, powrót 360 mAh -> zostaje usable - 526.7
    plan = _plan(526.7 + 22 / 3.6 * 30)          # 30 s spirali przy 22 A
    assert plan.spiral_time_s == pytest.approx(30, abs=0.1)
    assert not plan.full_music_possible
    assert not plan.success_possible


def test_plan_unreachable_target():
    plan = _plan(100, distance=5000)
    assert not plan.can_reach_and_return
    assert plan.spiral_time_s == 0
    assert not plan.success_possible
