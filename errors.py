class PreflightWarning(Exception):
    """Wspólna klasa bazowa dla ostrzeżeń przed startem."""

class LowBatteryWarning(PreflightWarning):
    pass

class DifferentLandingLocation(PreflightWarning):
    pass

class ArmingError(Exception):
    pass

class CommandError(Exception):
    """Autopilot odrzucił polecenie."""

class InFlightError(Exception):
    """Wspólna klasa dla problemów w trakcie lotu."""

class LowBatteryInFlight(InFlightError):
    pass

class LinkLostError(InFlightError):
    pass