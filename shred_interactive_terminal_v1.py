"""
SHRED interactive continuous-oscillation experiment
Version: 1.0

Workflow
--------
1. System starts resting.
2. Type "start" to zero, arm, and begin continuous oscillation.
3. At the idle prompt:
     - "go" starts one reaction-time trial.
     - "stop" safely stops Teensy and exits.
4. During a trial:
     - choose speed 1/2/3, mapped to different KD values
     - enter waveform/direction together, for example "1 r"
     - Mac waits a random 1-2 seconds
     - Teensy plays the waveform during the next complete matching travel
     - reaction timer starts on Teensy's WAVEFORM_STARTED event
     - press Enter once the direction is recognized
     - answer l/r
     - terminal reports reaction time and correctness

The serial reader and heartbeat run in background threads, so Teensy
remains connected while input() waits for the participant.
"""

from __future__ import annotations

import csv
import queue
import random
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Iterable

import serial
from serial import SerialException
from serial.tools import list_ports


# ============================================================
# CONNECTION
# ============================================================

PREFERRED_PORT = "/dev/cu.usbmodem105828301"
BAUD_RATE = 115200
READ_TIMEOUT_S = 0.05
COMMAND_TIMEOUT_S = 3.0
HEARTBEAT_PERIOD_S = 0.5


# ============================================================
# BASE MOTOR CONFIGURATION
# ============================================================

LEFT_DEG = -15.0
RIGHT_DEG = 15.0

KP = 1.5
PWM_MAX = 80

UPHILL_FACTOR = 1.6
DOWNHILL_MIN = 30

POSITION_TOL_DEG = 0.25
DWELL_MS = 100
MOVE_TIMEOUT_MS = 8000

STREAM_HZ = 50

MAG_SCALE = "AUTO"
CUSTOM1_PERIODS = 10.0


# ============================================================
# SPEED CONDITIONS
# ============================================================

# Higher KD adds damping and generally slows the basket.
# These are initial values and should be calibrated experimentally.
SPEED_KD = {
    1: 0.40,  # slow
    2: 0.20,  # medium/default
    3: 0.00,  # fast/original P-only behavior
}

DEFAULT_SPEED = 2


# ============================================================
# WAVEFORM CONDITIONS
# ============================================================

# Change these mappings to the two waveforms used in the study.
WAVEFORMS = {
    1: "cos",
    2: "custom1",
}

# r means the basket/magnet travel is left-to-right.
# l means the basket/magnet travel is right-to-left.
DIRECTION_TO_TEENSY = {
    "r": "L2R",
    "l": "R2L",
}

DIRECTION_LABEL = {
    "r": "left-to-right",
    "l": "right-to-left",
}


# ============================================================
# LOGGING
# ============================================================


def create_session_paths() -> tuple[Path, Path]:
    log_dir = Path.cwd() / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    raw_path = log_dir / f"shred_interactive_raw_{stamp}.csv"
    results_path = log_dir / f"shred_interactive_results_{stamp}.csv"

    return raw_path, results_path


# ============================================================
# SERIAL CLIENT
# ============================================================


class TeensyClient:
    def __init__(
        self,
        preferred_port: str | None,
        raw_log_path: Path,
    ) -> None:
        self.preferred_port = preferred_port
        self.raw_log_path = raw_log_path

        self.serial: serial.Serial | None = None

        self._send_lock = threading.Lock()
        self._log_lock = threading.Lock()

        self._response_queue: queue.Queue[str] = queue.Queue()

        self._event_condition = threading.Condition()
        self._events: deque[tuple[str, float]] = deque()

        self._running = threading.Event()
        self._fault = threading.Event()
        self._fault_line = ""

        self._reader_thread: threading.Thread | None = None
        self._heartbeat_thread: threading.Thread | None = None

        self._raw_file = None
        self._raw_writer = None

        self.latest_data: dict[str, str] = {}
        self._latest_data_lock = threading.Lock()

    @staticmethod
    def find_port(preferred_port: str | None) -> str:
        ports = [port.device for port in list_ports.comports()]

        if preferred_port and preferred_port in ports:
            return preferred_port

        candidates = [
            device
            for device in ports
            if "usbmodem" in device.lower()
        ]

        if len(candidates) == 1:
            return candidates[0]

        if not candidates:
            raise RuntimeError(
                "No Teensy USB modem port found. "
                f"Available ports: {ports}"
            )

        raise RuntimeError(
            "Multiple USB modem ports found. "
            f"Set PREFERRED_PORT: {candidates}"
        )

    def connect(self) -> None:
        port = self.find_port(self.preferred_port)

        print(f"Opening Teensy port: {port}")

        self.serial = serial.Serial(
            port=port,
            baudrate=BAUD_RATE,
            timeout=READ_TIMEOUT_S,
            write_timeout=1.0,
        )

        self._raw_file = self.raw_log_path.open(
            "w",
            newline="",
            encoding="utf-8",
        )

        self._raw_writer = csv.writer(self._raw_file)
        self._raw_writer.writerow(
            [
                "host_time_iso",
                "host_monotonic_s",
                "raw_line",
            ]
        )
        self._raw_file.flush()

        time.sleep(1.0)
        self.serial.reset_input_buffer()

        self._running.set()

        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="teensy-reader",
            daemon=True,
        )

        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            name="teensy-heartbeat",
            daemon=True,
        )

        self._reader_thread.start()
        self._heartbeat_thread.start()

    def close(self) -> None:
        self._running.clear()

        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)

        if self._heartbeat_thread is not None:
            self._heartbeat_thread.join(timeout=1.0)

        if self.serial is not None and self.serial.is_open:
            self.serial.close()

        if self._raw_file is not None:
            self._raw_file.flush()
            self._raw_file.close()

    def send_line(
        self,
        text: str,
        *,
        announce: bool = True,
    ) -> None:
        if self.serial is None or not self.serial.is_open:
            raise RuntimeError("Serial port is not open")

        if announce:
            print(f"Mac -> Teensy: {text}")

        payload = (text + "\n").encode("utf-8")

        with self._send_lock:
            self.serial.write(payload)
            self.serial.flush()

    def command(
        self,
        text: str,
        accepted_prefixes: Iterable[str],
        timeout_s: float = COMMAND_TIMEOUT_S,
    ) -> str:
        prefixes = tuple(accepted_prefixes)
        self.send_line(text)

        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            self.raise_if_faulted()

            remaining = max(
                0.01,
                deadline - time.monotonic(),
            )

            try:
                line = self._response_queue.get(
                    timeout=min(0.1, remaining)
                )
            except queue.Empty:
                continue

            if line.startswith("NACK"):
                raise RuntimeError(line)

            if line.startswith(prefixes):
                print(f"Teensy -> Mac: {line}")
                return line

        raise TimeoutError(
            f"No response to {text!r} starting with {prefixes}"
        )

    def wait_for_event(
        self,
        event_name: str,
        *,
        trial_id: int | None = None,
        timeout_s: float = 30.0,
    ) -> tuple[str, float]:
        deadline = time.monotonic() + timeout_s

        with self._event_condition:
            while True:
                self.raise_if_faulted()

                for index, (line, host_time) in enumerate(self._events):
                    event_match = f",{event_name}," in line
                    trial_match = (
                        trial_id is None
                        or f"trial={trial_id}" in line
                    )

                    if event_match and trial_match:
                        del self._events[index]
                        return line, host_time

                remaining = deadline - time.monotonic()

                if remaining <= 0:
                    raise TimeoutError(
                        "Timed out waiting for "
                        f"{event_name}, trial={trial_id}"
                    )

                self._event_condition.wait(
                    timeout=min(0.2, remaining)
                )

    def raise_if_faulted(self) -> None:
        if self._fault.is_set():
            raise RuntimeError(
                f"Teensy faulted: {self._fault_line}"
            )

    @staticmethod
    def _decode_line(raw: bytes) -> str:
        return raw.decode(
            "utf-8",
            errors="replace",
        ).strip()

    def _log_raw_line(
        self,
        line: str,
        host_monotonic: float,
    ) -> None:
        host_iso = (
            datetime.now()
            .astimezone()
            .isoformat(timespec="milliseconds")
        )

        with self._log_lock:
            self._raw_writer.writerow( # type: ignore
                [
                    host_iso,
                    f"{host_monotonic:.6f}",
                    line,
                ]
            )
            self._raw_file.flush() # type: ignore

    @staticmethod
    def _parse_data(line: str) -> dict[str, str]:
        fields = line.split(",")

        if len(fields) != 15 or fields[0] != "DATA":
            return {}

        names = [
            "record_type",
            "teensy_time_us",
            "state",
            "position_deg",
            "target_deg",
            "error_deg",
            "velocity_deg_s",
            "kd",
            "controller_pwm",
            "final_pwm",
            "minimum_pwm",
            "uphill",
            "magnet_pwm",
            "curve_index",
            "waveform_active",
        ]

        return dict(zip(names, fields, strict=True)) # type: ignore

    def _reader_loop(self) -> None:
        assert self.serial is not None

        while self._running.is_set():
            try:
                raw = self.serial.readline()
            except SerialException as error:
                self._fault_line = f"SERIAL_READ_ERROR: {error}"
                self._fault.set()
                return

            if not raw:
                continue

            line = self._decode_line(raw)

            if not line:
                continue

            host_monotonic = time.monotonic()

            self._log_raw_line(
                line,
                host_monotonic,
            )

            if line.startswith("DATA,"):
                parsed = self._parse_data(line)

                if parsed:
                    with self._latest_data_lock:
                        self.latest_data = parsed

                continue

            if line == "PONG":
                continue

            if line.startswith("FAULT,"):
                self._fault_line = line
                self._fault.set()

                print(f"\nTeensy -> Mac: {line}")

                with self._event_condition:
                    self._event_condition.notify_all()

                continue

            if line.startswith("EVENT,"):
                print(f"\nTeensy -> Mac: {line}")

                with self._event_condition:
                    self._events.append(
                        (line, host_monotonic)
                    )
                    self._event_condition.notify_all()

                continue

            if (
                line.startswith("ACK")
                or line.startswith("NACK")
                or line.startswith("CONFIG")
                or line.startswith("STATUS")
                or line.startswith("READY")
            ):
                self._response_queue.put(line)
                continue

            print(f"\nTeensy -> Mac: {line}")

    def _heartbeat_loop(self) -> None:
        while self._running.is_set():
            try:
                if self.serial is not None and self.serial.is_open:
                    self.send_line(
                        "PING",
                        announce=False,
                    )
            except (SerialException, RuntimeError) as error:
                self._fault_line = f"HEARTBEAT_ERROR: {error}"
                self._fault.set()
                return

            time.sleep(HEARTBEAT_PERIOD_S)


# ============================================================
# USER INPUT
# ============================================================


def read_speed_choice() -> int:
    while True:
        text = input(
            "\nChoose speed "
            "[1=slow, 2=medium, 3=fast]: "
        ).strip()

        if text in {"1", "2", "3"}:
            return int(text)

        print("Please enter 1, 2, or 3.")


def read_waveform_direction() -> tuple[int, str]:
    while True:
        text = input(
            "\nChoose waveform and direction "
            "(examples: '1 r' or '2 l'):\n"
            "  waveform 1 = cos\n"
            "  waveform 2 = custom1\n"
            "  r = left-to-right\n"
            "  l = right-to-left\n"
            "> "
        ).strip().lower()

        parts = text.split()

        if len(parts) != 2:
            print("Enter two values, such as: 1 r")
            continue

        waveform_text, direction = parts

        if waveform_text not in {"1", "2"}:
            print("Waveform must be 1 or 2.")
            continue

        if direction not in {"l", "r"}:
            print("Direction must be l or r.")
            continue

        return int(waveform_text), direction


def read_direction_guess() -> str:
    while True:
        guess = input(
            "Which direction was it? [l/r]: "
        ).strip().lower()

        if guess in {"l", "r"}:
            return guess

        print("Please enter l or r.")


# ============================================================
# RESULTS LOG
# ============================================================


RESULT_FIELDS = [
    "trial_id",
    "host_time_iso",
    "speed_choice",
    "kd",
    "waveform_choice",
    "waveform",
    "correct_direction_key",
    "physical_direction",
    "random_delay_s",
    "waveform_started_event",
    "reaction_time_s",
    "participant_guess",
    "correct",
    "waveform_ended_event",
]


def initialize_results_log(path: Path) -> None:
    with path.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=RESULT_FIELDS,
        )
        writer.writeheader()


def append_trial_result(
    path: Path,
    row: dict[str, object],
) -> None:
    with path.open(
        "a",
        newline="",
        encoding="utf-8",
    ) as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=RESULT_FIELDS,
        )
        writer.writerow(row)


# ============================================================
# SESSION WORKFLOW
# ============================================================


def send_base_configuration(
    client: TeensyClient,
) -> None:
    commands = [
        f"SET LEFT {LEFT_DEG}",
        f"SET RIGHT {RIGHT_DEG}",
        f"SET KP {KP}",
        f"SET PWM_MAX {PWM_MAX}",
        f"SET UPHILL_FACTOR {UPHILL_FACTOR}",
        f"SET DOWNHILL_MIN {DOWNHILL_MIN}",
        f"SET POSITION_TOL {POSITION_TOL_DEG}",
        f"SET DWELL_MS {DWELL_MS}",
        f"SET MOVE_TIMEOUT_MS {MOVE_TIMEOUT_MS}",
        f"SET MAG_SCALE {MAG_SCALE}",
        f"SET CUSTOM1_PERIODS {CUSTOM1_PERIODS}",
        f"SET STREAM_HZ {STREAM_HZ}",
    ]

    for command_text in commands:
        client.command(
            command_text,
            ("ACK SET",),
        )


def run_one_trial(
    client: TeensyClient,
    results_path: Path,
    trial_id: int,
) -> None:
    speed_choice = read_speed_choice()
    kd = SPEED_KD[speed_choice]

    # KD can change while the basket is oscillating.
    client.command(
        f"SET KD {kd}",
        ("ACK SET KD",),
    )

    waveform_choice, correct_key = read_waveform_direction()

    waveform = WAVEFORMS[waveform_choice]
    teensy_direction = DIRECTION_TO_TEENSY[correct_key]
    physical_direction = DIRECTION_LABEL[correct_key]

    random_delay = random.uniform(1.0, 2.0)

    print(
        "\nTrial configured."
        f"\n  speed={speed_choice}, KD={kd}"
        f"\n  waveform={waveform}"
        f"\n  direction={physical_direction}"
        f"\nWaiting a random {random_delay:.3f} s..."
    )

    # Background threads continue serial reading and heartbeat here.
    time.sleep(random_delay)

    client.command(
        "QUEUE WAVEFORM "
        f"{trial_id} "
        f"{waveform} "
        f"{teensy_direction}",
        ("ACK QUEUE WAVEFORM",),
    )

    print(
        "\nWaveform queued."
        "\nIt will begin at the next complete "
        f"{physical_direction} travel."
    )

    started_event, onset_host_time = client.wait_for_event(
        "WAVEFORM_STARTED",
        trial_id=trial_id,
        timeout_s=30.0,
    )

    print(
        "\nWAVEFORM IS ACTIVE."
        "\nPress Enter as soon as you know "
        "the travel direction."
    )

    input()

    reaction_time = time.monotonic() - onset_host_time

    participant_guess = read_direction_guess()
    correct = participant_guess == correct_key

    ended_event, _ = client.wait_for_event(
        "WAVEFORM_ENDED",
        trial_id=trial_id,
        timeout_s=15.0,
    )

    print(
        "\nTrial result:"
        f"\n  reaction time: {reaction_time:.3f} s"
        f"\n  answer: {participant_guess}"
        f"\n  correct direction: {correct_key}"
        f"\n  correct: {'YES' if correct else 'NO'}"
    )

    append_trial_result(
        results_path,
        {
            "trial_id": trial_id,
            "host_time_iso": (
                datetime.now()
                .astimezone()
                .isoformat(timespec="milliseconds")
            ),
            "speed_choice": speed_choice,
            "kd": kd,
            "waveform_choice": waveform_choice,
            "waveform": waveform,
            "correct_direction_key": correct_key,
            "physical_direction": physical_direction,
            "random_delay_s": f"{random_delay:.6f}",
            "waveform_started_event": started_event,
            "reaction_time_s": f"{reaction_time:.6f}",
            "participant_guess": participant_guess,
            "correct": correct,
            "waveform_ended_event": ended_event,
        },
    )


def main() -> int:
    raw_path, results_path = create_session_paths()
    initialize_results_log(results_path)

    client = TeensyClient(
        preferred_port=PREFERRED_PORT,
        raw_log_path=raw_path,
    )

    try:
        client.connect()

        client.command(
            "HELLO",
            ("ACK HELLO",),
        )

        send_base_configuration(client)

        client.command(
            "GET CONFIG",
            ("CONFIG",),
        )

        print(
            "\nSystem is resting."
            "\nKeep the magnet supply OFF for the "
            "first software-only validation."
            "\nPlace the basket at physical zero."
        )

        while True:
            initial_command = input(
                "\nType 'start' to begin oscillation "
                "or 'stop' to quit: "
            ).strip().lower()

            if initial_command == "stop":
                client.command(
                    "STOP",
                    ("ACK STOP",),
                )
                return 0

            if initial_command != "start":
                print("Please type start or stop.")
                continue

            default_kd = SPEED_KD[DEFAULT_SPEED]

            client.command(
                f"SET KD {default_kd}",
                ("ACK SET KD",),
            )

            client.command(
                "ZERO",
                ("ACK ZERO",),
            )

            client.command(
                "START STREAM",
                ("ACK START STREAM",),
            )

            client.command(
                "ARM",
                ("ACK ARM",),
            )

            input(
                "\nTurn ON motor power now."
                "\nThe magnet supply should still be OFF "
                "during the first validation."
                "\nConfirm the basket remains still while ARMED."
                "\nClear the mechanism and press Enter..."
            )

            client.command(
                "START",
                ("ACK START",),
            )

            print(
                "\nContinuous oscillation started."
                f"\nDefault speed={DEFAULT_SPEED}, "
                f"KD={default_kd}."
            )

            break

        trial_id = 1

        while True:
            client.raise_if_faulted()

            command_text = input(
                "\nType 'go' for a trial or "
                "'stop' to stop and quit: "
            ).strip().lower()

            if command_text == "stop":
                client.command(
                    "STOP",
                    ("ACK STOP",),
                )

                print(
                    "\nSystem stopped."
                    "\nTurn OFF motor and magnet power."
                    f"\nRaw log: {raw_path}"
                    f"\nResults: {results_path}"
                )

                return 0

            if command_text != "go":
                print("Please type go or stop.")
                continue

            run_one_trial(
                client,
                results_path,
                trial_id,
            )

            trial_id += 1

    except KeyboardInterrupt:
        print("\nCtrl+C received. Sending ESTOP.")

        try:
            client.send_line("ESTOP")
            time.sleep(0.2)
        except (SerialException, RuntimeError):
            pass

        print(
            "\nEmergency stop sent."
            "\nTurn OFF motor and magnet power."
            f"\nRaw log: {raw_path}"
            f"\nResults: {results_path}"
        )

        return 130

    except (
        SerialException,
        RuntimeError,
        TimeoutError,
        OSError,
        ValueError,
    ) as error:
        print(
            f"\nFAIL: {error}",
            file=sys.stderr,
        )

        try:
            client.send_line("ESTOP")
            time.sleep(0.2)
        except (SerialException, RuntimeError):
            pass

        print(
            "\nTurn OFF motor and magnet power."
            f"\nRaw log: {raw_path}"
            f"\nResults: {results_path}"
        )

        return 1

    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
