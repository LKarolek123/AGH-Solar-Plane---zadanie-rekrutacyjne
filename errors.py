"""Wyjątki używane w programie.

Zasada: Vehicle i pozostałe moduły tylko ZGŁASZAJĄ problemy (raise),
a o reakcji decyduje warstwa misji (mission.py) i main.py.
"""


# --- Przed startem (dron na ziemi, nic nie trzeba ratować) -----------------
class PreflightWarning(Exception):
    """Wspólna klasa bazowa dla ostrzeżeń przed startem."""

class LowBatteryWarning(PreflightWarning):
    pass

class DifferentLandingLocation(PreflightWarning):
    pass

class OperatorAbort(PreflightWarning):
    """Operator nie zgodził się na start."""

class MissionNotFeasible(PreflightWarning):
    """Misja nie mieści się w budżecie energii (sprawdzane przed startem)."""


# --- Polecenia do autopilota -----------------------------------------------
class ArmingError(Exception):
    pass

class CommandError(Exception):
    """Autopilot odrzucił polecenie."""


# --- W trakcie lotu ---------------------------------------------------------
class InFlightError(Exception):
    """Wspólna klasa dla problemów w trakcie lotu."""

class LowBatteryInFlight(InFlightError):
    """Bateria spadła poniżej twardego progu awaryjnego."""

class EnergyBudgetExceeded(InFlightError):
    """Zostało tylko tyle energii, ile potrzeba na powrót i lądowanie z zapasem."""

class LinkLostError(InFlightError):
    """Utracono łączność z autopilotem."""

class PilotOverrideError(InFlightError):
    """Ktoś (pilot, Mission Planner, failsafe) zmienił tryb lotu - nie walczymy z nim."""


# --- Głośnik ----------------------------------------------------------------
class SpeakerError(Exception):
    """Głośnika nie da się przygotować (brak pliku, brak biblioteki, brak urządzenia audio)."""

class MusicPlaybackError(InFlightError):
    """Muzyka nie gra albo przestała grać w trakcie misji."""
