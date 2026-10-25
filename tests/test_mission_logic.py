"""Testy logiki misji na atrapach - bez SITL."""
import time
from types import SimpleNamespace

import pytest

from errors import EnergyBudgetExceeded, SpeakerError, MusicPlaybackError
from functions import offset_point
from mission import EnergyGuard, KukurinMission
from speaker import Speaker

HOME = (50.0605, 19.9175, 0.0)


def fake_vehicle(distance_home_m, battery_pct, capacity=5000, consumed=None, current=20.0):
    lat, lon = offset_point(HOME[0], HOME[1], distance_home_m, 0)
    return SimpleNamespace(position=(lat, lon, 12.0), home=HOME, battery_status=battery_pct,
                           battery_capacity_mah=capacity, consumed_mah=consumed, current_a=current,
                           _current_samples=None)


# ---------------------------------------------------------------- EnergyGuard
def test_guard_allows_flight_with_enough_energy():
    guard = EnergyGuard(fly_current_a=20, speaker_current_a=2)
    guard(fake_vehicle(300, battery_pct=80))           # nie rzuca


def test_guard_forces_return_when_only_return_energy_left():
    guard = EnergyGuard(fly_current_a=20, speaker_current_a=2)
    # 5000 mAh * 12% = 600 mAh, zapas 500 -> zostaje 100 mAh, a powrót z 1000 m to ok. 1.2*20*124/3.6
    with pytest.raises(EnergyBudgetExceeded):
        guard(fake_vehicle(1000, battery_pct=12))


def test_disabled_guard_only_logs():
    guard = EnergyGuard(fly_current_a=20, speaker_current_a=2)
    guard.enabled = False
    guard(fake_vehicle(1000, battery_pct=12))          # nie rzuca


def test_guard_counts_virtual_speaker_consumption():
    guard = EnergyGuard(fly_current_a=20, speaker_current_a=36.0)   # 36 A = 10 mAh/s
    vehicle = fake_vehicle(10, battery_pct=90)
    guard(vehicle)
    guard.speaker_on = True
    time.sleep(0.3)
    guard(vehicle)
    assert guard.virtual_speaker_mah == pytest.approx(3.0, abs=0.6)


def test_guard_uses_higher_of_measured_and_live_current():
    guard = EnergyGuard(fly_current_a=20, speaker_current_a=0)
    vehicle = fake_vehicle(10, battery_pct=90, current=40.0)
    for _ in range(50):
        guard(vehicle)
    assert guard.return_current_a() > 35


# -------------------------------------------------------------------- Speaker
def test_simulated_speaker_plays_and_reports_progress():
    speaker = Speaker("simulated", "nie_ma_takiego_pliku.mp3")
    speaker.prepare()
    speaker.play()
    time.sleep(0.2)
    assert speaker.is_playing()
    assert speaker.elapsed_s() >= 0.15
    speaker.stop()
    assert not speaker.is_playing()


def test_speaker_fault_injection():
    speaker = Speaker("simulated", "x.mp3", fail_after_s=0.1)
    speaker.prepare()
    speaker.play()
    time.sleep(0.15)
    assert not speaker.is_playing()


def test_pygame_speaker_without_file_fails_before_flight(tmp_path):
    speaker = Speaker("pygame", str(tmp_path / "brak.mp3"))
    with pytest.raises(SpeakerError):
        speaker.prepare()


# ------------------------------------------------------ reakcja na awarię muzyki
def test_music_failure_restarts_once_then_aborts():
    speaker = Speaker("simulated", "x.mp3")
    speaker.prepare()
    speaker.play()
    speaker.is_playing = lambda: False                 # muzyka "nie gra"
    mission = KukurinMission.__new__(KukurinMission)
    mission.vehicle = fake_vehicle(0, battery_pct=90)
    mission.speaker = speaker
    mission.target = HOME
    mission._spiral_max_radius = 0.0
    mission._music_restarts = 0
    mission._speaker_validated = True
    mission.guard = EnergyGuard(20, 2)
    mission._spiral_duration = 60
    mission._warned_small_spiral = False

    mission._spiral_tick(1.0)                          # pierwsza awaria -> restart
    assert mission._music_restarts == 1
    with pytest.raises(MusicPlaybackError):
        mission._spiral_tick(2.0)                      # druga -> przerwanie misji
