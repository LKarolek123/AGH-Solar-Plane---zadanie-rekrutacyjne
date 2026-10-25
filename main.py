"""Uruchomienie misji.

    python main.py                      # normalnie, z pytaniami do operatora
    python main.py --yes                # bez pytań (np. przy testach scenariuszy)
    python main.py --speaker simulated  # bez dźwięku - głośnik tylko w logach
    python main.py --fault-music 20     # test awarii: muzyka "psuje się" po 20 s

Kody wyjścia: 0 - misja wykonana, 1 - niewykonana / przerwana, 2 - przerwana przez operatora.
"""
import os
os.environ['MAVLINK20'] = '1'

import argparse
import logging
import sys
import time
from pathlib import Path

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from conf import (
    MAVLINK_CONNECTION, MAVLINK_DIALECT, SOURCE_SYSTEM, HEARTBEAT_TIMEOUT_S,
    TARGET_LATITUDE, TARGET_LONGITUDE, MUSIC_FILE, SPEAKER_BACKEND,
)
from errors import PreflightWarning, SpeakerError, LinkLostError, PilotOverrideError
from mission import KukurinMission
from speaker import Speaker
from vehicle import Vehicle

log = logging.getLogger("drone")

TELEMETRY = [
    (mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1.0),
    (mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 0.5),     # częściej - do liczenia średniego prądu
    (mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 1.0),
    (mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 1.0),
    (mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 1.0),
]


def setup_logging():
    Path("logs").mkdir(exist_ok=True)
    log_file = Path("logs") / time.strftime("mission_%Y%m%d_%H%M%S.log")
    fmt = logging.Formatter("%(asctime)s %(levelname)-7s %(message)s", "%H:%M:%S")
    root = logging.getLogger("drone")
    root.setLevel(logging.INFO)
    for handler in (logging.StreamHandler(sys.stdout), logging.FileHandler(log_file, encoding="utf-8")):
        handler.setFormatter(fmt)
        root.addHandler(handler)
    return log_file


def parse_args():
    p = argparse.ArgumentParser(description="Misja: Kukurin nad Miasteczkiem Studenckim")
    p.add_argument("--connection", default=MAVLINK_CONNECTION, help="adres MAVLink (domyślnie z conf.py)")
    p.add_argument("--target", nargs=2, type=float, metavar=("LAT", "LON"),
                   default=(TARGET_LATITUDE, TARGET_LONGITUDE), help="współrzędne celu")
    p.add_argument("--speaker", choices=("pygame", "simulated"), default=SPEAKER_BACKEND)
    p.add_argument("--music", default=MUSIC_FILE, help="plik z muzyką")
    p.add_argument("--yes", action="store_true", help="nie pytaj operatora o zgodę")
    p.add_argument("--fault-music", type=float, metavar="S",
                   help="TEST: po S sekundach odtwarzanie przestaje działać")
    return p.parse_args()


def main():
    args = parse_args()
    log_file = setup_logging()
    log.info("Log misji: %s", log_file)

    try:
        conn = mavutil.mavlink_connection(
            args.connection,
            dialect=MAVLINK_DIALECT,
            source_system=SOURCE_SYSTEM,
            autoreconnect=True,
        )
    except ConnectionRefusedError:
        log.error("Nie można połączyć się z %s - czy symulacja SITL jest uruchomiona "
                  "i port nie jest zajęty przez poprzednie uruchomienie programu?", args.connection)
        return 1

    speaker = Speaker(args.speaker, args.music, volume=1.0, fail_after_s=args.fault_music)
    vehicle = None
    try:
        heartbeat = conn.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S)
        if heartbeat is None:
            raise TimeoutError('Brak heartbeat autopilota')
        conn.target_component = heartbeat.get_srcComponent()
        log.info("Połączono z autopilotem: system %d, komponent %d",
                 conn.target_system, conn.target_component)

        vehicle = Vehicle(conn)
        if args.yes:
            vehicle.ask_operator = lambda question: (log.info("%s -> TAK (--yes)", question) or True)
        for msg_id, interval in TELEMETRY:
            vehicle.request_message_interval(msg_id, interval)

        mission = KukurinMission(vehicle, speaker, args.target, auto_confirm=args.yes)
        result = mission.run()

        log.info("Wynik: %s | %s", "SUKCES" if result.success else "NIEPOWODZENIE", result.reason)
        for key, value in result.data.items():
            log.info("  %s = %s", key, value)
        return 0 if result.success and result.landed_home else 1

    except (PreflightWarning, SpeakerError) as e:
        # dron na ziemi - nic nie trzeba ratować
        log.warning("Start wstrzymany: %s", e)
        return 1

    except PilotOverrideError as e:
        # ktoś przejął drona - nie wysyłamy już żadnych poleceń
        log.warning("%s - oddaję kontrolę, program kończy działanie", e)
        return 1

    except LinkLostError as e:
        log.error("%s - próbuję RTL; jeśli polecenie nie dojdzie, drona sprowadzi failsafe autopilota", e)
        if vehicle is not None:
            vehicle.emergency_return()
        return 1

    except KeyboardInterrupt:
        log.warning("Przerwano przez operatora (Ctrl+C)")
        speaker.stop()
        if vehicle is not None and conn.motors_armed():
            vehicle.emergency_return()
        return 2

    except Exception as e:
        log.exception("Misja przerwana: %s: %s", type(e).__name__, e)
        speaker.stop()
        if vehicle is not None and conn.motors_armed():
            vehicle.emergency_return()
        return 1

    finally:
        speaker.close()
        try:
            conn.close()
        except Exception:        # po zerwanym połączeniu gniazda może już nie być
            pass


if __name__ == "__main__":
    sys.exit(main())
