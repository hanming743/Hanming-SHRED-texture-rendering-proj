"""
SHRED participant-window experiment
Version: 11.0-phase2-two-loadcell

Phase-2 study protocol
----------------------
- Exactly 8 units per participant.
- The researcher manually chooses S (slow) or F (fast) at the start of each unit.
- Exactly 4 slow units and 4 fast units are allowed; the program shows completed counts and blocks a fifth unit of either speed.
- The waveform is fixed to custom1 for every trial. There is no waveform selection.
- The old researcher-guided training/demo phase is removed.

Each unit contains 16 recorded trials:
1. Six feedback trials.
   - L/R are randomized.
   - Each direction appears at least twice, so valid counts are 2/4, 3/3, or 4/2.
   - After the participant answers, the screen shows actual direction, answer, and Correct/Incorrect.
   - Feedback remains visible until the participant presses Up Arrow. That same Up Arrow is treated as readiness for the next trial.
2. Ten no-feedback trials.
   - Exactly 5 L and 5 R, shuffled.
   - Behavior matches the previous recorded trials: the answer is recorded without revealing correctness.

Direction naming is intentionally inherited from the prior study:
- L = key "l" = Teensy direction R2L = destination is left.
- R = key "r" = Teensy direction L2R = destination is right.

Force recording
---------------
- Two independent NAU7802 load cells are tared together before motor motion begins.
- Both sensors are recorded for all 128 trials, including both feedback and no-feedback trials.
- The force CSV carries unit, phase/feedback condition, trial, direction, response, and correctness metadata in addition to sensor_id and force samples.

Safety
------
- Shift+Esc requests ESTOP. With pynput installed it can be system-wide; otherwise it works while the participant window has focus.
- Closing the participant window also requests ESTOP.
- Ctrl+C in the terminal remains an emergency fallback.
"""

from __future__ import annotations

import csv
import queue
import random
import sys
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable, TextIO

import serial
from serial import SerialException
from serial.tools import list_ports

try:
    from pynput import keyboard as pynput_keyboard
except ImportError:
    pynput_keyboard = None


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
DOWNHILL_MIN = 15
POSITION_TOL_DEG = 0.25
DWELL_MS = 20
MOVE_TIMEOUT_MS = 8000
STREAM_HZ = 50
MAG_SCALE: float | str = 255.0
CUSTOM1_PERIODS = 10.0


# ============================================================
# EXPERIMENT CONDITIONS
# ============================================================

SPEED_KD = {
    1: 0.04,  # slow
    2: 0.00,  # fast / original P-only behavior
}

SPEED_CODE_TO_CHOICE = {
    "S": 1,
    "F": 2,
}

SPEED_CHOICE_TO_CODE = {
    1: "S",
    2: "F",
}

SPEED_CHOICE_TO_NAME = {
    1: "slow",
    2: "fast",
}

FIXED_WAVEFORM_CHOICE = 2
FIXED_WAVEFORM = "custom1"

# Inherited naming from the previous study.
DIRECTION_TO_TEENSY = {
    "l": "L2R",  # destination left
    "r": "R2L",  # destination right
}

DIRECTION_DESTINATION = {
    "l": "L",
    "r": "R",
}

DIRECTION_PHYSICAL = {
    "l": "right-to-left",
    "r": "left-to-right",
}

TOTAL_UNITS = 8
UNITS_PER_SPEED = 4
FEEDBACK_TRIALS_PER_UNIT = 6
NO_FEEDBACK_TRIALS_PER_UNIT = 10
TRIALS_PER_UNIT = FEEDBACK_TRIALS_PER_UNIT + NO_FEEDBACK_TRIALS_PER_UNIT
TOTAL_TRIALS = TOTAL_UNITS * TRIALS_PER_UNIT

RANDOM_DELAY_MIN_S = 1.0
RANDOM_DELAY_MAX_S = 2.0
MISSED_STIMULUS_WARNING_S = 3.0
PRINT_BACKGROUND_EVENTS = False


# ============================================================
# DATA STRUCTURES
# ============================================================

@dataclass(frozen=True)
class UnitConfig:
    unit_id: int
    speed_choice: int
    unit_of_this_speed: int

    @property
    def kd(self) -> float:
        return SPEED_KD[self.speed_choice]

    @property
    def speed_code(self) -> str:
        return SPEED_CHOICE_TO_CODE[self.speed_choice]

    @property
    def unit_speed(self) -> str:
        return SPEED_CHOICE_TO_NAME[self.speed_choice]


@dataclass(frozen=True)
class TrialConfig:
    user_code: int
    trial_id: int
    global_trial: int
    unit_id: int
    unit_speed: str
    speed_code: str
    unit_of_this_speed: int
    trial_in_unit: int
    feedback_condition: str
    trial_in_phase: int
    phase_trial_count: int
    speed_choice: int
    direction_key: str
    unit_random_seed: int
    phase_random_seed: int

    @property
    def kd(self) -> float:
        return SPEED_KD[self.speed_choice]

    @property
    def waveform_choice(self) -> int:
        return FIXED_WAVEFORM_CHOICE

    @property
    def waveform(self) -> str:
        return FIXED_WAVEFORM

    @property
    def teensy_direction(self) -> str:
        return DIRECTION_TO_TEENSY[self.direction_key]

    @property
    def actual_direction(self) -> str:
        return DIRECTION_DESTINATION[self.direction_key]

    @property
    def physical_direction(self) -> str:
        return DIRECTION_PHYSICAL[self.direction_key]


# ============================================================
# LOGGING
# ============================================================

def create_session_paths(user_code: int) -> tuple[Path, Path, Path]:
    log_dir = Path.cwd() / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_path = log_dir / f"shred_ui_user{user_code}_raw_{stamp}.csv"
    results_path = log_dir / f"shred_ui_user{user_code}_results_{stamp}.csv"
    force_path = log_dir / f"shred_ui_user{user_code}_force_{stamp}.csv"
    return raw_path, results_path, force_path


RESULT_FIELDS = [
    "user_code",
    "unit_id",
    "unit_speed",
    "speed_code",
    "unit_of_this_speed",
    "trial_id",
    "global_trial",
    "trial_in_unit",
    "feedback_condition",
    "trial_in_phase",
    "phase_trial_count",
    "unit_random_seed",
    "phase_random_seed",
    "host_time_iso",
    "speed_choice",
    "kd",
    "waveform_choice",
    "waveform",
    "actual_direction",
    "correct_direction_key",
    "physical_direction",
    "teensy_direction",
    "random_delay_s",
    "ready_to_waveform_onset_s",
    "waveform_started_event",
    "reaction_time_s",
    "participant_answer",
    "participant_key",
    "correct",
    "waveform_end_event",
    "load_start_ack",
    "load_stop_ack",
    "load_dump_end_event",
]


FORCE_FIELDS = [
    "user_code",
    "unit_id",
    "unit_speed",
    "speed_code",
    "unit_of_this_speed",
    "trial_id",
    "global_trial",
    "trial_in_unit",
    "feedback_condition",
    "trial_in_phase",
    "phase_trial_count",
    "unit_random_seed",
    "phase_random_seed",
    "speed_choice",
    "kd",
    "waveform_choice",
    "waveform",
    "actual_direction",
    "correct_direction_key",
    "physical_direction",
    "teensy_direction",
    "participant_answer",
    "participant_key",
    "correct",
    "sensor_id",
    "sample_index",
    "teensy_time_us",
    "time_from_load_start_us",
    "time_from_load_start_s",
    "raw",
    "zero_offset",
    "delta",
    "basket_deg",
    "magnet_pwm",
    "waveform_active",
]


def initialize_results_log(path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=RESULT_FIELDS)
        writer.writeheader()


def append_trial_result(path: Path, row: dict[str, object]) -> None:
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=RESULT_FIELDS)
        writer.writerow(row)


# ============================================================
# SERIAL CLIENT
# ============================================================

class TeensyClient:
    def __init__(
        self,
        preferred_port: str | None,
        raw_log_path: Path,
        force_log_path: Path,
    ) -> None:
        self.preferred_port = preferred_port
        self.raw_log_path = raw_log_path
        self.force_log_path = force_log_path

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

        self._raw_file: TextIO | None = None
        self._raw_writer: Any | None = None

        self._force_file: TextIO | None = None
        self._force_writer: Any | None = None
        self._force_lock = threading.Lock()
        self._force_context_lock = threading.Lock()
        self._force_context: dict[str, object] | None = None

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

        self._force_file = self.force_log_path.open(
            "w",
            newline="",
            encoding="utf-8",
        )
        self._force_writer = csv.DictWriter(
            self._force_file,
            fieldnames=FORCE_FIELDS,
        )
        self._force_writer.writeheader()
        self._force_file.flush()

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

        if self._force_file is not None:
            self._force_file.flush()
            self._force_file.close()

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

    def wait_for_any_event(
        self,
        event_names: Iterable[str],
        *,
        trial_id: int | None = None,
        timeout_s: float = 30.0,
    ) -> tuple[str, float]:
        names = tuple(event_names)
        deadline = time.monotonic() + timeout_s

        with self._event_condition:
            while True:
                self.raise_if_faulted()

                for index, (line, host_time) in enumerate(self._events):
                    event_match = any(
                        f",{name}," in line
                        for name in names
                    )
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
                        f"{names}, trial={trial_id}"
                    )

                self._event_condition.wait(
                    timeout=min(0.2, remaining)
                )

    def wait_for_event(
        self,
        event_name: str,
        *,
        trial_id: int | None = None,
        timeout_s: float = 30.0,
    ) -> tuple[str, float]:
        return self.wait_for_any_event(
            (event_name,),
            trial_id=trial_id,
            timeout_s=timeout_s,
        )

    def raise_if_faulted(self) -> None:
        if self._fault.is_set():
            raise RuntimeError(
                f"Teensy faulted: {self._fault_line}"
            )

    def signal_local_abort(self, reason: str) -> None:
        """Abort local waits immediately, e.g. after a UI hard stop."""
        self._fault_line = reason
        self._fault.set()

        with self._event_condition:
            self._event_condition.notify_all()

    def set_force_context(self, trial: TrialConfig) -> None:
        context: dict[str, object] = {
            "user_code": trial.user_code,
            "unit_id": trial.unit_id,
            "unit_speed": trial.unit_speed,
            "speed_code": trial.speed_code,
            "unit_of_this_speed": trial.unit_of_this_speed,
            "trial_id": trial.trial_id,
            "global_trial": trial.global_trial,
            "trial_in_unit": trial.trial_in_unit,
            "feedback_condition": trial.feedback_condition,
            "trial_in_phase": trial.trial_in_phase,
            "phase_trial_count": trial.phase_trial_count,
            "unit_random_seed": trial.unit_random_seed,
            "phase_random_seed": trial.phase_random_seed,
            "speed_choice": trial.speed_choice,
            "kd": trial.kd,
            "waveform_choice": trial.waveform_choice,
            "waveform": trial.waveform,
            "actual_direction": trial.actual_direction,
            "correct_direction_key": trial.direction_key,
            "physical_direction": trial.physical_direction,
            "teensy_direction": trial.teensy_direction,
            "participant_answer": "",
            "participant_key": "",
            "correct": "",
        }

        with self._force_context_lock:
            self._force_context = context

    def update_force_outcome(
        self,
        participant_key: str,
        participant_answer: str,
        correct: bool,
    ) -> None:
        with self._force_context_lock:
            if self._force_context is None:
                return
            self._force_context["participant_key"] = participant_key
            self._force_context["participant_answer"] = participant_answer
            self._force_context["correct"] = correct

    def clear_force_context(self) -> None:
        with self._force_context_lock:
            self._force_context = None

    def _log_force_line(self, line: str) -> None:
        fields = line.split(",")

        # Firmware format:
        # LOAD_DATA,trial,sensor_id,sample_index,teensy_time_us,
        # time_from_start_us,raw,zero_offset,delta,basket_deg,
        # magnet_pwm,waveform_active
        if len(fields) != 12 or fields[0] != "LOAD_DATA":
            return

        data_trial_id = int(fields[1])
        sensor_id = int(fields[2])
        time_from_start_us = int(fields[5])

        with self._force_context_lock:
            context = (
                dict(self._force_context)
                if self._force_context is not None
                else {}
            )

        row: dict[str, object] = {
            "user_code": context.get("user_code", ""),
            "unit_id": context.get("unit_id", ""),
            "unit_speed": context.get("unit_speed", ""),
            "speed_code": context.get("speed_code", ""),
            "unit_of_this_speed": context.get("unit_of_this_speed", ""),
            "trial_id": data_trial_id,
            "global_trial": context.get("global_trial", ""),
            "trial_in_unit": context.get("trial_in_unit", ""),
            "feedback_condition": context.get("feedback_condition", ""),
            "trial_in_phase": context.get("trial_in_phase", ""),
            "phase_trial_count": context.get("phase_trial_count", ""),
            "unit_random_seed": context.get("unit_random_seed", ""),
            "phase_random_seed": context.get("phase_random_seed", ""),
            "speed_choice": context.get("speed_choice", ""),
            "kd": context.get("kd", ""),
            "waveform_choice": context.get("waveform_choice", ""),
            "waveform": context.get("waveform", ""),
            "actual_direction": context.get("actual_direction", ""),
            "correct_direction_key": context.get("correct_direction_key", ""),
            "physical_direction": context.get("physical_direction", ""),
            "teensy_direction": context.get("teensy_direction", ""),
            "participant_answer": context.get("participant_answer", ""),
            "participant_key": context.get("participant_key", ""),
            "correct": context.get("correct", ""),
            "sensor_id": sensor_id,
            "sample_index": fields[3],
            "teensy_time_us": fields[4],
            "time_from_load_start_us": time_from_start_us,
            "time_from_load_start_s": time_from_start_us / 1_000_000.0,
            "raw": fields[6],
            "zero_offset": fields[7],
            "delta": fields[8],
            "basket_deg": fields[9],
            "magnet_pwm": fields[10],
            "waveform_active": fields[11],
        }

        writer = self._force_writer
        force_file = self._force_file

        if writer is None or force_file is None:
            raise RuntimeError("Force log is not initialized")

        with self._force_lock:
            writer.writerow(row)
            force_file.flush()

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

        raw_writer = self._raw_writer
        raw_file = self._raw_file

        if raw_writer is None or raw_file is None:
            raise RuntimeError(
                "Raw serial log is not initialized"
            )

        with self._log_lock:
            raw_writer.writerow(
                [
                    host_iso,
                    f"{host_monotonic:.6f}",
                    line,
                ]
            )
            raw_file.flush()

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

        return dict(zip(names, fields))

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

            if line.startswith("LOAD_DATA,"):
                self._log_force_line(line)
                continue

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
                if PRINT_BACKGROUND_EVENTS:
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
                or line.startswith("LOAD_STATUS")
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
# GLOBAL HARD-STOP LISTENER
# ============================================================

class GlobalHardStopListener:
    """Optional system-wide Shift+Esc listener using pynput.

    Tk's bind_all only works while the participant app has keyboard focus.
    When pynput is installed and macOS grants keyboard-monitoring permission,
    this listener also catches Shift+Esc while the terminal is focused.
    """

    def __init__(self, callback: Any) -> None:
        self.callback = callback
        self.listener: Any | None = None
        self.shift_down = False

    def start(self) -> bool:
        if pynput_keyboard is None:
            return False

        def on_press(key: Any) -> None:
            if key in {
                pynput_keyboard.Key.shift,
                pynput_keyboard.Key.shift_l,
                pynput_keyboard.Key.shift_r,
            }:
                self.shift_down = True
                return

            if key == pynput_keyboard.Key.esc and self.shift_down:
                self.callback()

        def on_release(key: Any) -> None:
            if key in {
                pynput_keyboard.Key.shift,
                pynput_keyboard.Key.shift_l,
                pynput_keyboard.Key.shift_r,
            }:
                self.shift_down = False

        try:
            self.listener = pynput_keyboard.Listener(
                on_press=on_press,
                on_release=on_release,
            )
            self.listener.start()
            return True
        except Exception as error:
            print(
                "WARNING: global Shift+Esc listener could not start: "
                f"{error}"
            )
            self.listener = None
            return False

    def stop(self) -> None:
        if self.listener is not None:
            try:
                self.listener.stop()
            except Exception:
                pass
            self.listener = None

# ============================================================
# PARTICIPANT WINDOW
# ============================================================

class ParticipantWindow:
    def __init__(
        self,
        ui_queue: queue.Queue[tuple[str, str]],
        ready_press_queue: queue.Queue[float],
        response_queue: queue.Queue[tuple[str, float]],
        shutdown_event: threading.Event,
        hard_stop_callback: Any,
    ) -> None:
        self.ui_queue = ui_queue
        self.ready_press_queue = ready_press_queue
        self.response_queue = response_queue
        self.shutdown_event = shutdown_event
        self.hard_stop_callback = hard_stop_callback

        self.state = "IDLE"
        self.trial_generation = 0
        self._hard_stop_triggered = False

        self.root = tk.Tk()
        self.root.title("Sensation Direction Study")
        self.root.geometry("1000x650")
        self.root.minsize(800, 500)
        self.root.configure(bg="white")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.title_label = tk.Label(
            self.root,
            text="Sensation Direction Study",
            font=("Helvetica", 30, "bold"),
            bg="white",
            fg="black",
        )
        self.title_label.pack(pady=(60, 30))

        self.message_label = tk.Label(
            self.root,
            text="Waiting for the researcher to prepare the experiment.",
            font=("Helvetica", 24),
            justify="center",
            wraplength=840,
            bg="white",
            fg="black",
        )
        self.message_label.pack(expand=True, padx=70, pady=20)

        self.hint_label = tk.Label(
            self.root,
            text="",
            font=("Helvetica", 20, "bold"),
            justify="center",
            wraplength=860,
            bg="white",
            fg="black",
        )
        self.hint_label.pack(pady=(20, 70))

        self.root.bind("<Up>", self._on_up)
        self.root.bind("<Left>", lambda event: self._on_arrow("l"))
        self.root.bind("<Right>", lambda event: self._on_arrow("r"))
        self.root.bind_all("<Shift-Escape>", self._on_hard_stop)
        self.root.after(50, self._poll_ui_queue)

    def run(self) -> None:
        self.root.after(200, self._focus_window)
        self.root.mainloop()

    def _focus_window(self) -> None:
        try:
            self.root.lift()
            self.root.focus_force()
        except tk.TclError:
            return

    def _show(
        self,
        state: str,
        message: str,
        hint: str,
        *,
        focus: bool = False,
    ) -> None:
        self.state = state
        self.message_label.config(text=message)
        self.hint_label.config(text=hint)
        if focus:
            self.root.after(50, self._focus_window)

    def _on_up(self, event: tk.Event[Any]) -> str:
        del event

        if self.state in {"READY", "FEEDBACK"}:
            self.ready_press_queue.put(time.monotonic())
            self._show(
                "WAITING",
                "Please wait for the stimulus.",
                "Press ← or → as soon as you know the destination.",
            )

        return "break"

    def _on_arrow(self, response_key: str) -> str:
        if self.state != "DETECT":
            return "break"

        self.response_queue.put((response_key, time.monotonic()))
        self._show(
            "RESPONSE_SENT",
            "Response recorded.",
            "Please wait.",
        )
        return "break"

    def _on_hard_stop(self, event: tk.Event[Any] | None = None) -> str:
        del event
        if self._hard_stop_triggered:
            return "break"

        self._hard_stop_triggered = True
        self._show(
            "HARD_STOP",
            "EMERGENCY STOP",
            "Hardware stop requested. Turn off actuator power if needed.",
        )

        threading.Thread(
            target=self.hard_stop_callback,
            name="hard-stop-sender",
            daemon=True,
        ).start()

        self.root.after(800, self._destroy_after_hard_stop)
        return "break"

    def _destroy_after_hard_stop(self) -> None:
        self.shutdown_event.set()
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def _schedule_missed_stimulus_warning(self) -> None:
        generation = self.trial_generation
        delay_ms = int(MISSED_STIMULUS_WARNING_S * 1000)

        def show_warning() -> None:
            if generation == self.trial_generation and self.state == "DETECT":
                self._show(
                    "DETECT",
                    "You might have missed the stimulus.",
                    "Press ← or → as soon as you identify the destination.",
                    focus=True,
                )

        self.root.after(delay_ms, show_warning)

    def _poll_ui_queue(self) -> None:
        try:
            while True:
                command, payload = self.ui_queue.get_nowait()

                if command == "READY":
                    self.trial_generation += 1
                    self._show(
                        "READY",
                        "Press ↑ when you are ready.",
                        "←  Destination L          →  Destination R",
                        focus=True,
                    )

                elif command == "DETECT":
                    self._show(
                        "DETECT",
                        "Press the direction arrow as soon as you know the destination.",
                        "←  Destination L          →  Destination R",
                        focus=True,
                    )
                    self._schedule_missed_stimulus_warning()

                elif command == "FEEDBACK":
                    actual, answer, correct_text = payload.split("|", 2)
                    self._show(
                        "FEEDBACK",
                        f"Actual direction: {actual}\nYour answer: {answer}\n\n{correct_text}",
                        "Press ↑ when you are ready for the next trial.",
                        focus=True,
                    )

                elif command == "RECORDED":
                    self._show(
                        "RECORDED",
                        "Response recorded.",
                        "Please wait for the next trial.",
                    )

                elif command == "UNIT_COMPLETE":
                    self._show(
                        "UNIT_COMPLETE",
                        "This unit is complete.",
                        "Please wait for the researcher to prepare the next unit.",
                    )

                elif command == "IDLE":
                    self._show(
                        "IDLE",
                        "Waiting for the researcher to prepare the next unit.",
                        "",
                    )

                elif command == "FINISHED":
                    self._show(
                        "FINISHED",
                        "The experiment is complete.",
                        "Thank you.",
                    )

                elif command == "ERROR":
                    self._show(
                        "ERROR",
                        "The experiment has stopped because of a system error.",
                        payload,
                    )

                elif command == "CLOSE":
                    self.shutdown_event.set()
                    self.root.destroy()
                    return

        except queue.Empty:
            pass
        except tk.TclError:
            return

        if not self.shutdown_event.is_set():
            self.root.after(50, self._poll_ui_queue)

    def _on_close(self) -> None:
        self._on_hard_stop(None)


# ============================================================
# INPUT AND WORKFLOW HELPERS
# ============================================================

def read_terminal_line(prompt: str) -> str:
    """Read one terminal line without invoking GNU readline/libedit."""
    sys.stdout.write(prompt)
    sys.stdout.flush()
    line = sys.stdin.readline()
    if line == "":
        raise EOFError("Terminal input stream closed")
    return line.rstrip("\r\n")


def read_user_code_before_tk() -> int:
    while True:
        text = input("\nEnter integer user code: ").strip()
        try:
            value = int(text)
        except ValueError:
            print("User code must be an integer.")
            continue
        if value < 0:
            print("User code must be zero or greater.")
            continue
        return value


def parse_speed_selection(
    text: str,
    unit_id: int,
    completed_counts: dict[str, int],
) -> UnitConfig | None:
    cleaned = text.strip().upper()

    if cleaned == "STOP":
        return None

    aliases = {
        "S": "S",
        "SLOW": "S",
        "1": "S",
        "F": "F",
        "FAST": "F",
        "2": "F",
    }

    if cleaned not in aliases:
        raise ValueError("Enter S for slow, F for fast, or stop.")

    speed_code = aliases[cleaned]

    if completed_counts[speed_code] >= UNITS_PER_SPEED:
        raise ValueError(
            f"{speed_code} already has {UNITS_PER_SPEED}/{UNITS_PER_SPEED} completed units. "
            "Choose the other speed."
        )

    speed_choice = SPEED_CODE_TO_CHOICE[speed_code]
    return UnitConfig(
        unit_id=unit_id,
        speed_choice=speed_choice,
        unit_of_this_speed=completed_counts[speed_code] + 1,
    )


def speed_prompt(unit_id: int, completed_counts: dict[str, int]) -> str:
    return (
        f"\nChoose speed for unit {unit_id}/{TOTAL_UNITS} or type stop:\n"
        f"  S = slow   completed {completed_counts['S']}/{UNITS_PER_SPEED}\n"
        f"  F = fast   completed {completed_counts['F']}/{UNITS_PER_SPEED}\n"
        "> "
    )


def choose_unit_before_tk(
    unit_id: int,
    completed_counts: dict[str, int],
) -> UnitConfig | None:
    while True:
        text = input(speed_prompt(unit_id, completed_counts))
        try:
            return parse_speed_selection(text, unit_id, completed_counts)
        except ValueError as error:
            print(error)


def choose_unit_during_tk(
    unit_id: int,
    completed_counts: dict[str, int],
) -> UnitConfig | None:
    while True:
        text = read_terminal_line(speed_prompt(unit_id, completed_counts))
        try:
            return parse_speed_selection(text, unit_id, completed_counts)
        except ValueError as error:
            print(error)


def drain_queue(target_queue: queue.Queue[Any]) -> None:
    while True:
        try:
            target_queue.get_nowait()
        except queue.Empty:
            return


def wait_for_queue_item(
    target_queue: queue.Queue[Any],
    client: TeensyClient,
    shutdown_event: threading.Event,
) -> Any:
    while not shutdown_event.is_set():
        client.raise_if_faulted()
        try:
            return target_queue.get(timeout=0.1)
        except queue.Empty:
            continue
    raise RuntimeError("Experiment shutdown requested")


def sleep_with_safety_checks(
    duration_s: float,
    client: TeensyClient,
    shutdown_event: threading.Event,
) -> None:
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        if shutdown_event.is_set():
            raise RuntimeError("Experiment shutdown requested")
        client.raise_if_faulted()
        remaining = max(0.0, deadline - time.monotonic())
        time.sleep(min(0.05, remaining))


def send_base_configuration(client: TeensyClient) -> None:
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
        client.command(command_text, ("ACK SET",))


def recover_to_idle(client: TeensyClient) -> None:
    status_line = client.command("GET STATUS", ("STATUS",))
    if "state=FAULT" in status_line or "state=ESTOP" in status_line:
        client.command("CLEAR FAULT", ("ACK CLEAR FAULT",))
    elif "state=IDLE" not in status_line and "state=ARMED" not in status_line:
        client.command("STOP", ("ACK STOP",))


def make_feedback_sequence(seed: int) -> list[str]:
    """Six fair-random L/R trials, conditioned on at least two of each."""
    rng = random.Random(seed)
    while True:
        sequence = [rng.choice(("l", "r")) for _ in range(FEEDBACK_TRIALS_PER_UNIT)]
        l_count = sequence.count("l")
        r_count = sequence.count("r")
        if l_count >= 2 and r_count >= 2:
            return sequence


def make_no_feedback_sequence(seed: int) -> list[str]:
    sequence = ["l"] * 5 + ["r"] * 5
    rng = random.Random(seed)
    rng.shuffle(sequence)
    return sequence


# ============================================================
# RECORDED TRIALS
# ============================================================

def run_recorded_trial(
    client: TeensyClient,
    results_path: Path,
    trial: TrialConfig,
    ui_queue: queue.Queue[tuple[str, str]],
    ready_press_queue: queue.Queue[float],
    response_queue: queue.Queue[tuple[str, float]],
    shutdown_event: threading.Event,
    *,
    pre_ready_time: float | None = None,
) -> float | None:
    drain_queue(response_queue)

    if pre_ready_time is None:
        drain_queue(ready_press_queue)
        ui_queue.put(("READY", ""))
        ready_press_time = float(
            wait_for_queue_item(ready_press_queue, client, shutdown_event)
        )
    else:
        # This Up Arrow was pressed on the previous feedback screen and is
        # intentionally reused as readiness for the current trial.
        ready_press_time = float(pre_ready_time)

    print(
        "\nRecorded trial:"
        f"\n  user={trial.user_code}"
        f"\n  unit={trial.unit_id}/{TOTAL_UNITS} ({trial.unit_speed}, {trial.speed_code})"
        f"\n  unit of this speed={trial.unit_of_this_speed}/{UNITS_PER_SPEED}"
        f"\n  trial={trial.trial_id}/{TOTAL_TRIALS}"
        f"\n  trial in unit={trial.trial_in_unit}/{TRIALS_PER_UNIT}"
        f"\n  condition={trial.feedback_condition}"
        f"\n  trial in phase={trial.trial_in_phase}/{trial.phase_trial_count}"
        f"\n  KD={trial.kd}"
        f"\n  waveform={trial.waveform}"
        f"\n  actual destination={trial.actual_direction}"
        f"\n  Teensy direction={trial.teensy_direction}"
    )

    client.set_force_context(trial)
    load_start_ack = client.command(
        f"LOAD START {trial.trial_id}",
        ("ACK LOAD START",),
    )

    random_delay = random.uniform(RANDOM_DELAY_MIN_S, RANDOM_DELAY_MAX_S)
    print(f"Participant ready. Hidden delay: {random_delay:.3f} s")
    sleep_with_safety_checks(random_delay, client, shutdown_event)

    client.command(
        f"QUEUE WAVEFORM {trial.trial_id} {trial.waveform} {trial.teensy_direction}",
        ("ACK QUEUE WAVEFORM",),
    )

    started_event, onset_host_time = client.wait_for_event(
        "WAVEFORM_STARTED",
        trial_id=trial.trial_id,
        timeout_s=30.0,
    )

    ui_queue.put(("DETECT", ""))
    response_key, response_time = wait_for_queue_item(
        response_queue,
        client,
        shutdown_event,
    )

    reaction_time = float(response_time) - onset_host_time
    participant_answer = DIRECTION_DESTINATION[response_key]
    correct = response_key == trial.direction_key
    correct_text = "Correct!" if correct else "Incorrect"

    # Add response outcome to the context before force rows are dumped.
    client.update_force_outcome(
        participant_key=response_key,
        participant_answer=participant_answer,
        correct=correct,
    )

    if trial.feedback_condition == "feedback":
        ui_queue.put(
            (
                "FEEDBACK",
                f"{trial.actual_direction}|{participant_answer}|{correct_text}",
            )
        )
    else:
        ui_queue.put(("RECORDED", ""))

    waveform_end_event, _ = client.wait_for_any_event(
        ("WAVEFORM_ENDED", "WAVEFORM_TIMEOUT"),
        trial_id=trial.trial_id,
        timeout_s=15.0,
    )

    load_stop_ack = client.command("LOAD STOP", ("ACK LOAD STOP",))
    load_dump_end_event, _ = client.wait_for_event(
        "LOAD_DUMP_END",
        trial_id=trial.trial_id,
        timeout_s=30.0,
    )
    client.clear_force_context()

    ready_to_onset = onset_host_time - ready_press_time

    print(
        "\nTrial result:"
        f"\n  actual={trial.actual_direction}"
        f"\n  answer={participant_answer}"
        f"\n  correct={'YES' if correct else 'NO'}"
        f"\n  reaction time={reaction_time:.3f} s"
    )

    append_trial_result(
        results_path,
        {
            "user_code": trial.user_code,
            "unit_id": trial.unit_id,
            "unit_speed": trial.unit_speed,
            "speed_code": trial.speed_code,
            "unit_of_this_speed": trial.unit_of_this_speed,
            "trial_id": trial.trial_id,
            "global_trial": trial.global_trial,
            "trial_in_unit": trial.trial_in_unit,
            "feedback_condition": trial.feedback_condition,
            "trial_in_phase": trial.trial_in_phase,
            "phase_trial_count": trial.phase_trial_count,
            "unit_random_seed": trial.unit_random_seed,
            "phase_random_seed": trial.phase_random_seed,
            "host_time_iso": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "speed_choice": trial.speed_choice,
            "kd": trial.kd,
            "waveform_choice": trial.waveform_choice,
            "waveform": trial.waveform,
            "actual_direction": trial.actual_direction,
            "correct_direction_key": trial.direction_key,
            "physical_direction": trial.physical_direction,
            "teensy_direction": trial.teensy_direction,
            "random_delay_s": f"{random_delay:.6f}",
            "ready_to_waveform_onset_s": f"{ready_to_onset:.6f}",
            "waveform_started_event": started_event,
            "reaction_time_s": f"{reaction_time:.6f}",
            "participant_answer": participant_answer,
            "participant_key": response_key,
            "correct": correct,
            "waveform_end_event": waveform_end_event,
            "load_start_ack": load_start_ack,
            "load_stop_ack": load_stop_ack,
            "load_dump_end_event": load_dump_end_event,
        },
    )

    if trial.feedback_condition == "feedback":
        # Feedback remains on screen until Up is pressed. That Up becomes
        # readiness for the next trial; do not ask for a second readiness key.
        return float(
            wait_for_queue_item(ready_press_queue, client, shutdown_event)
        )

    return None


def run_unit(
    client: TeensyClient,
    results_path: Path,
    user_code: int,
    unit: UnitConfig,
    first_trial_id: int,
    ui_queue: queue.Queue[tuple[str, str]],
    ready_press_queue: queue.Queue[float],
    response_queue: queue.Queue[tuple[str, float]],
    shutdown_event: threading.Event,
) -> int:
    client.command(f"SET KD {unit.kd}", ("ACK SET KD",))

    unit_seed = random.SystemRandom().randrange(0, 2**32)
    seed_rng = random.Random(unit_seed)
    feedback_seed = seed_rng.randrange(0, 2**32)
    no_feedback_seed = seed_rng.randrange(0, 2**32)

    feedback_sequence = make_feedback_sequence(feedback_seed)
    no_feedback_sequence = make_no_feedback_sequence(no_feedback_seed)

    print(
        "\nUnit starting:"
        f"\n  unit={unit.unit_id}/{TOTAL_UNITS}"
        f"\n  speed={unit.unit_speed} ({unit.speed_code}), KD={unit.kd}"
        f"\n  unit of this speed={unit.unit_of_this_speed}/{UNITS_PER_SPEED}"
        f"\n  waveform={FIXED_WAVEFORM}"
        f"\n  unit seed={unit_seed}"
        f"\n  feedback seed={feedback_seed}"
        f"\n  feedback sequence={feedback_sequence} "
        f"(L={feedback_sequence.count('l')}, R={feedback_sequence.count('r')})"
        f"\n  no-feedback seed={no_feedback_seed}"
        f"\n  no-feedback sequence={no_feedback_sequence}"
    )

    next_trial_id = first_trial_id
    pre_ready_time: float | None = None

    # Phase 1: six feedback trials.
    for trial_in_phase, direction_key in enumerate(feedback_sequence, start=1):
        if shutdown_event.is_set():
            raise RuntimeError("Experiment shutdown requested")

        trial_in_unit = trial_in_phase
        trial = TrialConfig(
            user_code=user_code,
            trial_id=next_trial_id,
            global_trial=next_trial_id,
            unit_id=unit.unit_id,
            unit_speed=unit.unit_speed,
            speed_code=unit.speed_code,
            unit_of_this_speed=unit.unit_of_this_speed,
            trial_in_unit=trial_in_unit,
            feedback_condition="feedback",
            trial_in_phase=trial_in_phase,
            phase_trial_count=FEEDBACK_TRIALS_PER_UNIT,
            speed_choice=unit.speed_choice,
            direction_key=direction_key,
            unit_random_seed=unit_seed,
            phase_random_seed=feedback_seed,
        )

        pre_ready_time = run_recorded_trial(
            client,
            results_path,
            trial,
            ui_queue,
            ready_press_queue,
            response_queue,
            shutdown_event,
            pre_ready_time=pre_ready_time,
        )
        next_trial_id += 1

    # Phase 2: ten no-feedback trials, exactly five L and five R.
    for trial_in_phase, direction_key in enumerate(no_feedback_sequence, start=1):
        if shutdown_event.is_set():
            raise RuntimeError("Experiment shutdown requested")

        trial_in_unit = FEEDBACK_TRIALS_PER_UNIT + trial_in_phase
        trial = TrialConfig(
            user_code=user_code,
            trial_id=next_trial_id,
            global_trial=next_trial_id,
            unit_id=unit.unit_id,
            unit_speed=unit.unit_speed,
            speed_code=unit.speed_code,
            unit_of_this_speed=unit.unit_of_this_speed,
            trial_in_unit=trial_in_unit,
            feedback_condition="no_feedback",
            trial_in_phase=trial_in_phase,
            phase_trial_count=NO_FEEDBACK_TRIALS_PER_UNIT,
            speed_choice=unit.speed_choice,
            direction_key=direction_key,
            unit_random_seed=unit_seed,
            phase_random_seed=no_feedback_seed,
        )

        pre_ready_time = run_recorded_trial(
            client,
            results_path,
            trial,
            ui_queue,
            ready_press_queue,
            response_queue,
            shutdown_event,
            pre_ready_time=pre_ready_time,
        )
        next_trial_id += 1

    ui_queue.put(("UNIT_COMPLETE", ""))
    return next_trial_id


# ============================================================
# SESSION LOOP
# ============================================================

def experiment_session_loop(
    client: TeensyClient,
    results_path: Path,
    user_code: int,
    first_unit: UnitConfig,
    completed_counts: dict[str, int],
    ui_queue: queue.Queue[tuple[str, str]],
    ready_press_queue: queue.Queue[float],
    response_queue: queue.Queue[tuple[str, float]],
    shutdown_event: threading.Event,
    graceful_stop_event: threading.Event,
    hard_stop_event: threading.Event,
    outcome: dict[str, int],
) -> None:
    unit = first_unit
    next_trial_id = 1

    try:
        while not shutdown_event.is_set():
            next_trial_id = run_unit(
                client,
                results_path,
                user_code,
                unit,
                next_trial_id,
                ui_queue,
                ready_press_queue,
                response_queue,
                shutdown_event,
            )

            completed_counts[unit.speed_code] += 1
            print(
                "\nUnit complete."
                f"\n  Slow completed: {completed_counts['S']}/{UNITS_PER_SPEED}"
                f"\n  Fast completed: {completed_counts['F']}/{UNITS_PER_SPEED}"
                f"\n  Total completed: {sum(completed_counts.values())}/{TOTAL_UNITS}"
            )

            if shutdown_event.is_set():
                break

            if sum(completed_counts.values()) >= TOTAL_UNITS:
                if completed_counts != {"S": UNITS_PER_SPEED, "F": UNITS_PER_SPEED}:
                    raise RuntimeError(
                        "Internal unit-count error: expected exactly 4 slow and 4 fast units."
                    )

                client.command("STOP", ("ACK STOP",))
                graceful_stop_event.set()
                ui_queue.put(("FINISHED", ""))
                print(
                    "\nAll 8 units are complete."
                    "\nTurn OFF motor and magnet power."
                    f"\nRaw log: {client.raw_log_path}"
                    f"\nResults: {results_path}"
                    f"\nForce: {client.force_log_path}"
                )
                time.sleep(1.0)
                shutdown_event.set()
                ui_queue.put(("CLOSE", ""))
                return

            ui_queue.put(("IDLE", ""))
            next_unit_id = sum(completed_counts.values()) + 1
            next_unit = choose_unit_during_tk(next_unit_id, completed_counts)

            if next_unit is None:
                client.command("STOP", ("ACK STOP",))
                graceful_stop_event.set()
                shutdown_event.set()
                print("\nExperiment session ended normally by researcher.")
                print(
                    "Turn OFF motor and magnet power."
                    f"\nRaw log: {client.raw_log_path}"
                    f"\nResults: {results_path}"
                    f"\nForce: {client.force_log_path}"
                )
                ui_queue.put(("CLOSE", ""))
                return

            unit = next_unit

    except (
        SerialException,
        RuntimeError,
        TimeoutError,
        OSError,
        ValueError,
        EOFError,
    ) as error:
        if hard_stop_event.is_set():
            outcome["code"] = 130
            print("\nHARD STOP: Shift+Esc emergency stop was triggered.")
        else:
            outcome["code"] = 1
            print(f"\nFAIL: {error}", file=sys.stderr)
            try:
                client.send_line("ESTOP")
                time.sleep(0.2)
            except (SerialException, RuntimeError):
                pass
            ui_queue.put(("ERROR", str(error)))
            time.sleep(1.0)

        shutdown_event.set()
        ui_queue.put(("CLOSE", ""))


# ============================================================
# MAIN
# ============================================================

def main() -> int:
    user_code = read_user_code_before_tk()
    raw_path, results_path, force_path = create_session_paths(user_code)
    initialize_results_log(results_path)

    client = TeensyClient(
        preferred_port=PREFERRED_PORT,
        raw_log_path=raw_path,
        force_log_path=force_path,
    )

    shutdown_event = threading.Event()
    graceful_stop_event = threading.Event()
    hard_stop_event = threading.Event()
    outcome = {"code": 0}
    motion_started = False

    completed_counts = {"S": 0, "F": 0}

    ui_queue: queue.Queue[tuple[str, str]] = queue.Queue()
    ready_press_queue: queue.Queue[float] = queue.Queue()
    response_queue: queue.Queue[tuple[str, float]] = queue.Queue()

    experiment_thread: threading.Thread | None = None
    global_hard_stop_listener: GlobalHardStopListener | None = None

    try:
        client.connect()
        client.command("HELLO", ("ACK HELLO",))
        recover_to_idle(client)
        send_base_configuration(client)
        client.command("GET CONFIG", ("CONFIG",))

        load_status = client.command("LOAD STATUS", ("LOAD_STATUS",))
        if "ready=YES" not in load_status:
            raise RuntimeError("Both load cells must be ready: " + load_status)

        input(
            "\nMake sure BOTH load cells are unloaded and untouched."
            "\nPress Enter to tare both load cells..."
        )
        tare_ack = client.command(
            "LOAD TARE",
            ("ACK LOAD TARE",),
            timeout_s=8.0,
        )
        print(f"Two-load-cell tare complete: {tare_ack}")

        first_unit = choose_unit_before_tk(1, completed_counts)
        if first_unit is None:
            client.command("STOP", ("ACK STOP",))
            graceful_stop_event.set()
            return 0

        print(
            "\nSelected first unit:"
            f"\n  speed={first_unit.unit_speed} ({first_unit.speed_code})"
            f"\n  KD={first_unit.kd}"
            f"\n  waveform={FIXED_WAVEFORM}"
        )

        print(
            "\nSystem is resting."
            f"\nUser code: {user_code}"
            "\nPlace the basket at physical zero."
        )

        while True:
            initial_command = input(
                "\nType 'start' to begin oscillation or 'stop' to quit: "
            ).strip().lower()

            if initial_command == "stop":
                client.command("STOP", ("ACK STOP",))
                graceful_stop_event.set()
                return 0

            if initial_command != "start":
                print("Please type start or stop.")
                continue

            client.command(f"SET KD {first_unit.kd}", ("ACK SET KD",))
            client.command("ZERO", ("ACK ZERO",))
            client.command("START STREAM", ("ACK START STREAM",))
            client.command("ARM", ("ACK ARM",))

            input(
                "\nTurn ON motor and magnet power now."
                "\nClear the mechanism and press Enter..."
            )

            client.command("START", ("ACK START",))
            motion_started = True

            print(
                "\nContinuous oscillation started."
                f"\nUnit 1 speed={first_unit.unit_speed} ({first_unit.speed_code}), KD={first_unit.kd}."
                f"\nAll trials use waveform={FIXED_WAVEFORM}."
                "\nParticipant-window hard stop: Shift+Esc."
            )
            break

        def hard_stop_callback() -> None:
            if hard_stop_event.is_set():
                return
            hard_stop_event.set()
            client.signal_local_abort("LOCAL_HARD_STOP_SHIFT_ESC")
            try:
                client.send_line("ESTOP")
                time.sleep(0.2)
            except (SerialException, RuntimeError):
                pass
            shutdown_event.set()

        global_hard_stop_listener = GlobalHardStopListener(hard_stop_callback)
        global_hotkey_active = global_hard_stop_listener.start()

        if global_hotkey_active:
            print("Global Shift+Esc hard stop is active.")
        else:
            print(
                "WARNING: global Shift+Esc is not active. "
                "Shift+Esc still works while the participant window has focus, "
                "and Ctrl+C remains available in the terminal."
            )

        participant_window = ParticipantWindow(
            ui_queue,
            ready_press_queue,
            response_queue,
            shutdown_event,
            hard_stop_callback,
        )

        experiment_thread = threading.Thread(
            target=experiment_session_loop,
            args=(
                client,
                results_path,
                user_code,
                first_unit,
                completed_counts,
                ui_queue,
                ready_press_queue,
                response_queue,
                shutdown_event,
                graceful_stop_event,
                hard_stop_event,
                outcome,
            ),
            name="experiment-session",
            daemon=True,
        )

        experiment_thread.start()
        participant_window.run()

        if experiment_thread.is_alive():
            experiment_thread.join(timeout=1.0)

        return outcome["code"]

    except KeyboardInterrupt:
        print("\nCtrl+C received. Sending ESTOP.")
        outcome["code"] = 130
        shutdown_event.set()
        try:
            client.send_line("ESTOP")
            time.sleep(0.2)
        except (SerialException, RuntimeError):
            pass
        return 130

    except (
        SerialException,
        RuntimeError,
        TimeoutError,
        OSError,
        ValueError,
        EOFError,
    ) as error:
        print(f"\nFAIL: {error}", file=sys.stderr)
        outcome["code"] = 1
        shutdown_event.set()
        try:
            if motion_started:
                client.send_line("ESTOP")
            else:
                client.send_line("STOP")
            time.sleep(0.2)
        except (SerialException, RuntimeError):
            pass
        return 1

    finally:
        shutdown_event.set()

        if motion_started and not graceful_stop_event.is_set():
            try:
                client.send_line("ESTOP", announce=False)
                time.sleep(0.1)
            except (SerialException, RuntimeError):
                pass

        if global_hard_stop_listener is not None:
            global_hard_stop_listener.stop()

        client.close()

        print(
            f"\nRaw log: {raw_path}"
            f"\nResults: {results_path}"
            f"\nForce: {force_path}"
        )


if __name__ == "__main__":
    raise SystemExit(main())
