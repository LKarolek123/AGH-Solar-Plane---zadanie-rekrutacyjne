"""Głośnik drona.

W SITL nie ma fizycznego głośnika, więc:
  - backend "pygame" odtwarza plik na komputerze, na którym działa program,
  - backend "simulated" tylko loguje, co "gra" (np. gdy nie ma karty dźwiękowej).

Pobór prądu głośnika nie jest widoczny w baterii SITL - liczymy go wirtualnie w misji
(SPEAKER_CURRENT_A z conf.py).
"""
import logging
import os
import time

from errors import SpeakerError

log = logging.getLogger("drone.speaker")


class Speaker:
    def __init__(self, backend, music_file, volume=1.0, fail_after_s=None):
        """
        backend      - "pygame" albo "simulated"
        music_file   - ścieżka do pliku audio
        volume       - 0.0 .. 1.0 ("na full" = 1.0)
        fail_after_s - tylko do testów: po tylu sekundach odtwarzanie "psuje się"
        """
        self.backend = backend
        self.music_file = music_file
        self.volume = volume
        self.fail_after_s = fail_after_s
        self._pygame = None
        self._started_at = None
        self._playing = False

    # ------------------------------------------------------------------ setup
    def prepare(self):
        """Sprawdza głośnik przed startem. Rzuca SpeakerError, jeśli nie da się grać."""
        if self.backend == "simulated":
            log.info(f"[GŁOŚNIK] tryb symulowany - '{self.music_file}' będzie tylko logowany")
            return
        if self.backend != "pygame":
            raise SpeakerError(f"Nieznany backend głośnika: {self.backend}")

        if not os.path.isfile(self.music_file):
            raise SpeakerError(f"Brak pliku z muzyką: {self.music_file}")
        try:
            os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
            import pygame
        except ImportError as e:
            raise SpeakerError("Brak biblioteki pygame (pip install pygame)") from e
        try:
            pygame.mixer.init()
            pygame.mixer.music.load(self.music_file)
        except Exception as e:      # pygame.error, brak urządzenia audio itp.
            raise SpeakerError(f"Nie udało się przygotować odtwarzania: {e}") from e
        self._pygame = pygame
        log.info(f"[GŁOŚNIK] gotowy: {self.music_file}")

    # --------------------------------------------------------------- odtwarzanie
    def play(self):
        self._started_at = time.monotonic()
        self._playing = True
        if self._pygame is not None:
            self._pygame.mixer.music.set_volume(self.volume)
            self._pygame.mixer.music.play()
        log.info(f"[GŁOŚNIK] start: {os.path.basename(self.music_file)}, głośność {self.volume:.0%}")

    def stop(self):
        if self._pygame is not None:
            self._pygame.mixer.music.stop()
        if self._playing:
            log.info("[GŁOŚNIK] stop")
        self._playing = False

    def elapsed_s(self):
        """Ile sekund utworu już zagrało (0, jeśli nie gra)."""
        if not self._playing or self._started_at is None:
            return 0.0
        if self._pygame is not None:
            pos_ms = self._pygame.mixer.music.get_pos()
            return max(pos_ms, 0) / 1000
        return time.monotonic() - self._started_at

    def is_playing(self):
        """Czy muzyka faktycznie gra."""
        if not self._playing:
            return False
        if self.fail_after_s is not None and time.monotonic() - self._started_at >= self.fail_after_s:
            return False
        if self._pygame is not None:
            return bool(self._pygame.mixer.music.get_busy())
        return True

    def close(self):
        self.stop()
        if self._pygame is not None:
            try:
                self._pygame.mixer.quit()
            except Exception:
                pass
