"""
SHRED participant-window experiment
Version: 9.0-loadcell

Experiment organization
-----------------------
- The researcher enters an integer user code before any basket motion.
- The researcher selects one speed and one magnet pattern for each section.
- Only two speed conditions are used: slow and fast.
- The participant first completes a researcher-guided training/demo phase for
  that fixed speed/pattern.
- During training, the researcher temporarily takes over the participant-window
  keyboard: Left Arrow triggers the existing left-direction stimulus, Right
  Arrow triggers the existing right-direction stimulus, and Up Arrow ends training.
- Training directions are not pre-generated, typed in the terminal, or displayed
  by the program. The researcher knows the direction from the arrow they press
  and can verbally inform the participant after each stimulus.
- The recorded section then contains equal numbers of right-to-left and
  left-to-right trials, with only direction order randomized.
- TRIALS_PER_EXPERIMENT_PER_DIRECTION defaults to 15, giving 30 recorded
  trials per section.

Participant training
--------------------
1. The participant presses Up Arrow to begin learning the section stimulus.
2. The researcher temporarily takes over the participant-window keyboard.
3. Left Arrow triggers the existing left-direction stimulus; Right Arrow
   triggers the existing right-direction stimulus.
4. The program does not display or announce the training direction.
5. After a stimulus ends, the researcher may verbally tell the participant the
   direction, then press another arrow to present another example.
6. The researcher presses Up Arrow when training is complete and gives the keyboard
   back to the participant. The recorded experiment then begins.

Recorded trial
--------------
1. Press Up Arrow when ready.
2. Wait through the hidden random delay.
3. As soon as the stimulus direction is recognized, press Left or Right.
4. Reaction time is measured directly from stimulus onset to that arrow press.
5. There is no second readiness key and no "I don't know" response.

Force recording
---------------
- One NAU7802 load cell is tared before motor motion begins.
- Force recording starts when the participant presses Up Arrow to begin a
  recorded trial, before the hidden random delay.
- Recording continues through the stimulus and response, then stops after the
  waveform ends.
- Teensy buffers the 80 SPS samples in RAM and dumps them after the trial so
  serial logging cannot disturb motor/magnet timing during the stimulus.
- A separate force CSV is written for later analysis and joining by trial_id.

Safety
------
- Shift+Esc sends an immediate ESTOP request. With optional pynput installed,
  it is system-wide; otherwise it works while the participant window is focused.
- Closing the participant window is also treated as an emergency stop.
- Ctrl+C in the terminal remains an emergency fallback.

Tkinter stays on the main thread. After Tk starts, researcher terminal input
uses sys.stdin.readline() rather than input() to avoid the macOS Tcl notifier
crash observed in earlier versions.
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
DWELL_MS = 100
MOVE_TIMEOUT_MS = 8000

STREAM_HZ = 50

# A fixed scale of 255 makes the "full" curve actually use maximum PWM.
MAG_SCALE: float | str = 255.0
CUSTOM1_PERIODS = 10.0


# ============================================================
# EXPERIMENT CONDITIONS
# ============================================================

# Higher KD adds damping and usually slows the basket.
SPEED_KD = {
    1: 0.04,  # slow; increase this KD to make the slow condition slower
    2: 0.00,  # fast / original P-only behavior
}

# The Teensy firmware must support both names below.
WAVEFORMS = {
    1: "full",
    2: "custom1",
}

# Physical-direction keys used by both training and experiment responses.
# This uses the latest corrected physical mapping from the v6 setup.
DIRECTION_TO_TEENSY = {
    "r": "L2R",  # physical left-to-right
    "l": "R2L",  # physical right-to-left
}

DIRECTION_LABEL = {
    "r": "left-to-right",
    "l": "right-to-left",
}

PRINT_BACKGROUND_EVENTS = False

# Hidden onset delay for recorded trials. Speed/pattern stay fixed within a
# section; only left/right ordering is randomized across trials.
RANDOM_DELAY_MIN_S = 1.0
RANDOM_DELAY_MAX_S = 2.0
MISSED_STIMULUS_WARNING_S = 3.0

# Each experiment section contains this many recorded trials PER direction.
# Default: 15 RTL + 15 LTR = 30 recorded trials per section.
TRIALS_PER_EXPERIMENT_PER_DIRECTION = 15

# Training stimulus IDs are kept far away from recorded trial IDs.
# Training has no fixed number or pre-generated direction sequence in v7.
# The researcher controls each training stimulus from the participant window.
TRAINING_TRIAL_ID_BASE = 900000


# ============================================================
# DATA STRUCTURES
# ============================================================


@dataclass(frozen=True)
class SectionConfig:
    section_id: int
    speed_choice: int
    waveform_choice: int

    @property
    def kd(self) -> float:
        return SPEED_KD[self.speed_choice]

    @property
    def waveform(self) -> str:
        return WAVEFORMS[self.waveform_choice]


@dataclass(frozen=True)
class TrialConfig:
    user_code: int
    trial_id: int
    section_id: int
    trial_in_section: int
    speed_choice: int
    waveform_choice: int
    direction_key: str
    section_random_seed: int

    @property
    def kd(self) -> float:
        return SPEED_KD[self.speed_choice]

    @property
    def waveform(self) -> str:
        return WAVEFORMS[self.waveform_choice]

    @property
    def teensy_direction(self) -> str:
        return DIRECTION_TO_TEENSY[self.direction_key]

    @property
    def physical_direction(self) -> str:
        return DIRECTION_LABEL[self.direction_key]


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
    "section_id",
    "trial_id",
    "trial_in_section",
    "trials_per_direction",
    "section_random_seed",
    "host_time_iso",
    "speed_choice",
    "kd",
    "waveform_choice",
    "waveform",
    "correct_direction_key",
    "physical_direction",
    "teensy_direction",
    "random_delay_s",
    "ready_to_waveform_onset_s",
    "waveform_started_event",
    "reaction_time_s",
    "participant_response",
    "participant_key",
    "correct",
    "waveform_end_event",
    "load_start_ack",
    "load_stop_ack",
    "load_dump_end_event",
]


FORCE_FIELDS = [
    "user_code",
    "section_id",
    "trial_id",
    "trial_in_section",
    "speed_choice",
    "kd",
    "waveform_choice",
    "waveform",
    "correct_direction_key",
    "physical_direction",
    "teensy_direction",
    "sample_index",
    "teensy_time_us",
    "time_from_load_start_us",
    "raw",
    "zero_offset",
    "delta",
    "basket_deg",
    "magnet_pwm",
    "waveform_active",
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
            "section_id": trial.section_id,
            "trial_id": trial.trial_id,
            "trial_in_section": trial.trial_in_section,
            "speed_choice": trial.speed_choice,
            "kd": trial.kd,
            "waveform_choice": trial.waveform_choice,
            "waveform": trial.waveform,
            "correct_direction_key": trial.direction_key,
            "physical_direction": trial.physical_direction,
            "teensy_direction": trial.teensy_direction,
        }

        with self._force_context_lock:
            self._force_context = context

    def clear_force_context(self) -> None:
        with self._force_context_lock:
            self._force_context = None

    def _log_force_line(self, line: str) -> None:
        fields = line.split(",")

        if len(fields) != 11 or fields[0] != "LOAD_DATA":
            return

        data_trial_id = int(fields[1])

        with self._force_context_lock:
            context = (
                dict(self._force_context)
                if self._force_context is not None
                else {}
            )

        row: dict[str, object] = {
            "user_code": context.get("user_code", ""),
            "section_id": context.get("section_id", ""),
            "trial_id": data_trial_id,
            "trial_in_section": context.get("trial_in_section", ""),
            "speed_choice": context.get("speed_choice", ""),
            "kd": context.get("kd", ""),
            "waveform_choice": context.get("waveform_choice", ""),
            "waveform": context.get("waveform", ""),
            "correct_direction_key": context.get(
                "correct_direction_key", ""
            ),
            "physical_direction": context.get("physical_direction", ""),
            "teensy_direction": context.get("teensy_direction", ""),
            "sample_index": fields[2],
            "teensy_time_us": fields[3],
            "time_from_load_start_us": fields[4],
            "raw": fields[5],
            "zero_offset": fields[6],
            "delta": fields[7],
            "basket_deg": fields[8],
            "magnet_pwm": fields[9],
            "waveform_active": fields[10],
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
        training_intro_queue: queue.Queue[float],
        training_control_queue: queue.Queue[tuple[str, float]],
        ready_press_queue: queue.Queue[float],
        response_queue: queue.Queue[tuple[str, float]],
        shutdown_event: threading.Event,
        hard_stop_callback: Any,
    ) -> None:
        self.ui_queue = ui_queue
        self.training_intro_queue = training_intro_queue
        self.training_control_queue = training_control_queue
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
        self.message_label.pack(
            expand=True,
            padx=70,
            pady=20,
        )

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

        if self.state == "SECTION_INTRO":
            self.training_intro_queue.put(time.monotonic())
            self._show(
                "TRAINING_DEMOS",
                "Your researcher will show you a few different stimuli and will inform you of the direction when each one ends.",
                "Please pay attention to how each direction feels.",
            )

        elif self.state == "TRAINING_DEMOS":
            # During training, the researcher owns the participant-window
            # keyboard. Up Arrow means "training is finished".
            self.training_control_queue.put(
                ("done", time.monotonic())
            )
            self._show(
                "TRAINING_DONE",
                "Training complete.",
                "Please return the keyboard to the participant. "
                "The recorded experiment will begin next.",
            )

        elif self.state == "READY":
            self.ready_press_queue.put(time.monotonic())
            self._show(
                "WAITING",
                "Please wait.\n\nThe stimulus will show up within 3 seconds.",
                "Press ← or → as soon as you figure out the direction.",
            )
            self._schedule_missed_stimulus_warning()

        return "break"

    def _on_arrow(self, response_key: str) -> str:
        if self.state == "TRAINING_DEMOS":
            # Researcher-controlled training:
            # Left Arrow -> existing "l" stimulus
            # Right Arrow -> existing "r" stimulus
            self.training_control_queue.put(
                (response_key, time.monotonic())
            )

            # Lock out additional training key presses until the current
            # one-travel stimulus has finished.
            self.state = "TRAINING_BUSY"
            return "break"

        if self.state != "DETECT":
            return "break"

        self.response_queue.put(
            (response_key, time.monotonic())
        )

        self._show(
            "RECORDED",
            "Response recorded.",
            "Please wait for the next trial.",
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
            if (
                generation == self.trial_generation
                and self.state in {"WAITING", "DETECT"}
            ):
                self._show(
                    self.state,
                    "You might have missed the stimulus.",
                    "Stay ready and press ← or → as soon as you identify the direction.",
                    focus=True,
                )

        self.root.after(delay_ms, show_warning)

    def _poll_ui_queue(self) -> None:
        try:
            while True:
                command, payload = self.ui_queue.get_nowait()

                if command == "SECTION_INTRO":
                    self._show(
                        "SECTION_INTRO",
                        "You are going to be shown the stimuli for this section of the experiment.",
                        "Press ↑ when you are ready.",
                        focus=True,
                    )

                elif command == "TRAINING":
                    self._show(
                        "TRAINING_DEMOS",
                        "Your researcher will show you a few different stimuli "
                        "and will inform you of the direction when each one ends.",
                        "Please pay attention to how each direction feels.",
                        focus=True,
                    )

                elif command == "READY":
                    self.trial_generation += 1
                    self._show(
                        "READY",
                        "Press ↑ when you are ready.\n\n"
                        "The stimulus will show up within 3 seconds.\n\n"
                        "Press the direction arrow as soon as you figure out which direction.",
                        "←  Right-to-left          →  Left-to-right",
                        focus=True,
                    )

                elif command == "DETECT":
                    self._show(
                        "DETECT",
                        "Press the direction arrow as soon as you figure out which direction.",
                        "←  Right-to-left          →  Left-to-right",
                        focus=True,
                    )

                elif command == "SECTION_COMPLETE":
                    self._show(
                        "SECTION_COMPLETE",
                        "This section of the experiment is complete.",
                        "Please wait for the researcher.",
                    )

                elif command == "IDLE":
                    self._show(
                        "IDLE",
                        "Waiting for the researcher to prepare the next section.",
                        "",
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


def parse_section_selection(
    text: str,
    section_id: int,
) -> SectionConfig | None:
    cleaned = text.strip().lower()

    if cleaned == "stop":
        return None

    parts = cleaned.split()

    if len(parts) != 2:
        raise ValueError(
            "Enter two numbers: <speed> <pattern>, for example: 1 2"
        )

    try:
        speed_choice = int(parts[0])
        waveform_choice = int(parts[1])
    except ValueError as error:
        raise ValueError("Speed and pattern must be integers.") from error

    if speed_choice not in SPEED_KD:
        raise ValueError("Speed must be 1 or 2.")

    if waveform_choice not in WAVEFORMS:
        raise ValueError("Pattern must be 1 or 2.")

    return SectionConfig(
        section_id=section_id,
        speed_choice=speed_choice,
        waveform_choice=waveform_choice,
    )


def choose_section_before_tk(section_id: int) -> SectionConfig | None:
    while True:
        text = input(
            "\nChoose experiment section <speed> <pattern> or type stop:\n"
            "  speed:   1=slow, 2=fast\n"
            "  pattern: 1=full maximum, 2=custom1\n"
            "  example: 1 2\n"
            "> "
        )

        try:
            return parse_section_selection(text, section_id)
        except ValueError as error:
            print(error)


def choose_section_during_tk(section_id: int) -> SectionConfig | None:
    while True:
        text = read_terminal_line(
            "\nChoose next experiment section <speed> <pattern> or type stop:\n"
            "  speed:   1=slow, 2=fast\n"
            "  pattern: 1=full maximum, 2=custom1\n"
            "  example: 1 2\n"
            "> "
        )

        try:
            return parse_section_selection(text, section_id)
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
        client.command(
            command_text,
            ("ACK SET",),
        )


def recover_to_idle(client: TeensyClient) -> None:
    status_line = client.command(
        "GET STATUS",
        ("STATUS",),
    )

    if (
        "state=FAULT" in status_line
        or "state=ESTOP" in status_line
    ):
        client.command(
            "CLEAR FAULT",
            ("ACK CLEAR FAULT",),
        )
    elif (
        "state=IDLE" not in status_line
        and "state=ARMED" not in status_line
    ):
        client.command(
            "STOP",
            ("ACK STOP",),
        )


def make_direction_sequence(seed: int) -> list[str]:
    sequence = (
        ["l"] * TRIALS_PER_EXPERIMENT_PER_DIRECTION
        + ["r"] * TRIALS_PER_EXPERIMENT_PER_DIRECTION
    )

    rng = random.Random(seed)
    rng.shuffle(sequence)
    return sequence


# ============================================================
# TRAINING PHASE
# ============================================================


def run_training_phase(
    client: TeensyClient,
    section: SectionConfig,
    ui_queue: queue.Queue[tuple[str, str]],
    training_intro_queue: queue.Queue[float],
    training_control_queue: queue.Queue[tuple[str, float]],
    shutdown_event: threading.Event,
) -> None:
    drain_queue(training_intro_queue)
    drain_queue(training_control_queue)

    client.command(
        f"SET KD {section.kd}",
        ("ACK SET KD",),
    )

    print(
        "\nSection configured:"
        f"\n  section={section.section_id}"
        f"\n  speed={section.speed_choice}, KD={section.kd}"
        f"\n  pattern={section.waveform_choice} ({section.waveform})"
        "\n  recorded directions=right-to-left and left-to-right"
    )

    # Participant presses Up Arrow once to indicate readiness for training.
    ui_queue.put(("SECTION_INTRO", ""))

    wait_for_queue_item(
        training_intro_queue,
        client,
        shutdown_event,
    )

    # From this point until Up Arrow is pressed, the researcher temporarily
    # controls the participant-window keyboard.
    ui_queue.put(("TRAINING", ""))

    print(
        "\nTraining mode active."
        "\n  Researcher now controls the participant-window keyboard."
        "\n  Left Arrow  -> trigger the existing left-direction stimulus."
        "\n  Right Arrow -> trigger the existing right-direction stimulus."
        "\n  Up Arrow    -> finish training and return the keyboard to the participant."
        "\n  The program will NOT display or print the training direction."
    )

    training_count = 0

    while not shutdown_event.is_set():
        client.raise_if_faulted()

        action, _action_time = wait_for_queue_item(
            training_control_queue,
            client,
            shutdown_event,
        )

        if action == "done":
            print(
                "\nTraining complete. "
                "Return the keyboard to the participant."
            )
            return

        if action not in {"l", "r"}:
            continue

        training_count += 1

        training_trial_id = (
            TRAINING_TRIAL_ID_BASE
            + section.section_id * 1000
            + training_count
        )

        teensy_direction = DIRECTION_TO_TEENSY[action]

        print(
            f"\nTraining stimulus {training_count}: queued."
        )

        client.command(
            "QUEUE WAVEFORM "
            f"{training_trial_id} "
            f"{section.waveform} "
            f"{teensy_direction}",
            ("ACK QUEUE WAVEFORM",),
        )

        client.wait_for_event(
            "WAVEFORM_STARTED",
            trial_id=training_trial_id,
            timeout_s=30.0,
        )

        client.wait_for_any_event(
            ("WAVEFORM_ENDED", "WAVEFORM_TIMEOUT"),
            trial_id=training_trial_id,
            timeout_s=15.0,
        )

        # Deliberately do not reveal the direction here.
        print(
            f"Training stimulus {training_count}: complete. "
            "Researcher may verbally inform the participant, "
            "then press another arrow when ready."
        )

        # Re-enable training arrow/Up input after the current one-travel
        # stimulus has fully ended.
        ui_queue.put(("TRAINING", ""))

    raise RuntimeError("Experiment shutdown requested")


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
) -> None:
    drain_queue(ready_press_queue)
    drain_queue(response_queue)

    print(
        "\nRecorded trial:"
        f"\n  user={trial.user_code}"
        f"\n  section={trial.section_id}"
        f"\n  trial={trial.trial_id}"
        f"\n  trial in section={trial.trial_in_section}"
        f"\n  speed={trial.speed_choice}, KD={trial.kd}"
        f"\n  pattern={trial.waveform_choice} ({trial.waveform})"
        f"\n  physical direction={trial.physical_direction}"
        f"\n  Teensy direction={trial.teensy_direction}"
    )

    ui_queue.put(("READY", ""))

    ready_press_time = float(
        wait_for_queue_item(
            ready_press_queue,
            client,
            shutdown_event,
        )
    )

    client.set_force_context(trial)
    load_start_ack = client.command(
        f"LOAD START {trial.trial_id}",
        ("ACK LOAD START",),
    )

    random_delay = random.uniform(
        RANDOM_DELAY_MIN_S,
        RANDOM_DELAY_MAX_S,
    )

    print(
        f"Participant ready. Hidden delay: {random_delay:.3f} s"
    )

    sleep_with_safety_checks(
        random_delay,
        client,
        shutdown_event,
    )

    client.command(
        "QUEUE WAVEFORM "
        f"{trial.trial_id} "
        f"{trial.waveform} "
        f"{trial.teensy_direction}",
        ("ACK QUEUE WAVEFORM",),
    )

    started_event, onset_host_time = client.wait_for_event(
        "WAVEFORM_STARTED",
        trial_id=trial.trial_id,
        timeout_s=30.0,
    )

    # No second readiness key in v8. The direction key itself is the reaction-time event.
    ui_queue.put(("DETECT", ""))

    response_key, response_time = wait_for_queue_item(
        response_queue,
        client,
        shutdown_event,
    )

    reaction_time = float(response_time) - onset_host_time

    waveform_end_event, _ = client.wait_for_any_event(
        ("WAVEFORM_ENDED", "WAVEFORM_TIMEOUT"),
        trial_id=trial.trial_id,
        timeout_s=15.0,
    )

    load_stop_ack = client.command(
        "LOAD STOP",
        ("ACK LOAD STOP",),
    )

    load_dump_end_event, _ = client.wait_for_event(
        "LOAD_DUMP_END",
        trial_id=trial.trial_id,
        timeout_s=30.0,
    )
    client.clear_force_context()

    participant_response = DIRECTION_LABEL[response_key]
    correct = response_key == trial.direction_key
    ready_to_onset = onset_host_time - ready_press_time

    print(
        "\nTrial result:"
        f"\n  user={trial.user_code}"
        f"\n  section={trial.section_id}"
        f"\n  trial={trial.trial_id}"
        f"\n  speed={trial.speed_choice}"
        f"\n  pattern={trial.waveform_choice} ({trial.waveform})"
        f"\n  reaction time={reaction_time:.3f} s"
        f"\n  participant response={participant_response}"
        f"\n  correct direction={trial.physical_direction}"
        f"\n  correct={'YES' if correct else 'NO'}"
    )

    append_trial_result(
        results_path,
        {
            "user_code": trial.user_code,
            "section_id": trial.section_id,
            "trial_id": trial.trial_id,
            "trial_in_section": trial.trial_in_section,
            "trials_per_direction": TRIALS_PER_EXPERIMENT_PER_DIRECTION,
            "section_random_seed": trial.section_random_seed,
            "host_time_iso": (
                datetime.now()
                .astimezone()
                .isoformat(timespec="milliseconds")
            ),
            "speed_choice": trial.speed_choice,
            "kd": trial.kd,
            "waveform_choice": trial.waveform_choice,
            "waveform": trial.waveform,
            "correct_direction_key": trial.direction_key,
            "physical_direction": trial.physical_direction,
            "teensy_direction": trial.teensy_direction,
            "random_delay_s": f"{random_delay:.6f}",
            "ready_to_waveform_onset_s": f"{ready_to_onset:.6f}",
            "waveform_started_event": started_event,
            "reaction_time_s": f"{reaction_time:.6f}",
            "participant_response": participant_response,
            "participant_key": response_key,
            "correct": correct,
            "waveform_end_event": waveform_end_event,
            "load_start_ack": load_start_ack,
            "load_stop_ack": load_stop_ack,
            "load_dump_end_event": load_dump_end_event,
        },
    )


def run_recorded_section(
    client: TeensyClient,
    results_path: Path,
    user_code: int,
    section: SectionConfig,
    first_trial_id: int,
    ui_queue: queue.Queue[tuple[str, str]],
    ready_press_queue: queue.Queue[float],
    response_queue: queue.Queue[tuple[str, float]],
    shutdown_event: threading.Event,
) -> int:
    section_seed = random.SystemRandom().randrange(0, 2**32)
    direction_sequence = make_direction_sequence(section_seed)

    print(
        "\nRecorded section starting:"
        f"\n  section={section.section_id}"
        f"\n  speed={section.speed_choice}, KD={section.kd}"
        f"\n  pattern={section.waveform_choice} ({section.waveform})"
        f"\n  trials per direction={TRIALS_PER_EXPERIMENT_PER_DIRECTION}"
        f"\n  total recorded trials={len(direction_sequence)}"
        f"\n  random seed={section_seed}"
        f"\n  randomized directions={direction_sequence}"
    )

    next_trial_id = first_trial_id

    for trial_in_section, direction_key in enumerate(
        direction_sequence,
        start=1,
    ):
        if shutdown_event.is_set():
            raise RuntimeError("Experiment shutdown requested")

        trial = TrialConfig(
            user_code=user_code,
            trial_id=next_trial_id,
            section_id=section.section_id,
            trial_in_section=trial_in_section,
            speed_choice=section.speed_choice,
            waveform_choice=section.waveform_choice,
            direction_key=direction_key,
            section_random_seed=section_seed,
        )

        run_recorded_trial(
            client,
            results_path,
            trial,
            ui_queue,
            ready_press_queue,
            response_queue,
            shutdown_event,
        )

        next_trial_id += 1

    ui_queue.put(("SECTION_COMPLETE", ""))
    return next_trial_id


# ============================================================
# SESSION LOOP
# ============================================================


def experiment_session_loop(
    client: TeensyClient,
    results_path: Path,
    user_code: int,
    first_section: SectionConfig,
    ui_queue: queue.Queue[tuple[str, str]],
    training_intro_queue: queue.Queue[float],
    training_control_queue: queue.Queue[tuple[str, float]],
    ready_press_queue: queue.Queue[float],
    response_queue: queue.Queue[tuple[str, float]],
    shutdown_event: threading.Event,
    graceful_stop_event: threading.Event,
    hard_stop_event: threading.Event,
    outcome: dict[str, int],
) -> None:
    section = first_section
    next_trial_id = 1

    try:
        while not shutdown_event.is_set():
            run_training_phase(
                client,
                section,
                ui_queue,
                training_intro_queue,
                training_control_queue,
                shutdown_event,
            )

            next_trial_id = run_recorded_section(
                client,
                results_path,
                user_code,
                section,
                next_trial_id,
                ui_queue,
                ready_press_queue,
                response_queue,
                shutdown_event,
            )

            if shutdown_event.is_set():
                break

            ui_queue.put(("IDLE", ""))

            next_section = choose_section_during_tk(
                section.section_id + 1
            )

            if next_section is None:
                client.command(
                    "STOP",
                    ("ACK STOP",),
                )
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

            section = next_section

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

    ui_queue: queue.Queue[tuple[str, str]] = queue.Queue()
    training_intro_queue: queue.Queue[float] = queue.Queue()
    training_control_queue: queue.Queue[tuple[str, float]] = queue.Queue()
    ready_press_queue: queue.Queue[float] = queue.Queue()
    response_queue: queue.Queue[tuple[str, float]] = queue.Queue()

    experiment_thread: threading.Thread | None = None
    global_hard_stop_listener: GlobalHardStopListener | None = None

    try:
        client.connect()

        client.command(
            "HELLO",
            ("ACK HELLO",),
        )

        recover_to_idle(client)
        send_base_configuration(client)

        client.command(
            "GET CONFIG",
            ("CONFIG",),
        )

        load_status = client.command(
            "LOAD STATUS",
            ("LOAD_STATUS",),
        )
        if "ready=YES" not in load_status:
            raise RuntimeError(
                "Load cell is not ready: " + load_status
            )

        input(
            "\nMake sure the load cell is unloaded and untouched."
            "\nPress Enter to tare the load cell..."
        )
        tare_ack = client.command(
            "LOAD TARE",
            ("ACK LOAD TARE",),
            timeout_s=8.0,
        )
        print(f"Load cell tare complete: {tare_ack}")

        first_section = choose_section_before_tk(section_id=1)

        if first_section is None:
            client.command(
                "STOP",
                ("ACK STOP",),
            )
            graceful_stop_event.set()
            return 0

        print(
            "\nSelected section:"
            f"\n  speed={first_section.speed_choice}"
            f"\n  pattern={first_section.waveform_choice} ({first_section.waveform})"
            f"\n  conditions used: {first_section.speed_choice} "
            f"{first_section.waveform_choice} l and "
            f"{first_section.speed_choice} {first_section.waveform_choice} r"
        )

        print(
            "\nSystem is resting."
            f"\nUser code: {user_code}"
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
                graceful_stop_event.set()
                return 0

            if initial_command != "start":
                print("Please type start or stop.")
                continue

            client.command(
                f"SET KD {first_section.kd}",
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
                "\nClear the mechanism and press Enter..."
            )

            client.command(
                "START",
                ("ACK START",),
            )

            motion_started = True

            print(
                "\nContinuous oscillation started."
                f"\nSection speed={first_section.speed_choice}, "
                f"KD={first_section.kd}."
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

        global_hard_stop_listener = GlobalHardStopListener(
            hard_stop_callback
        )
        global_hotkey_active = global_hard_stop_listener.start()

        if global_hotkey_active:
            print("Global Shift+Esc hard stop is active.")
        else:
            print(
                "WARNING: global Shift+Esc is not active. "
                "Shift+Esc still works while the participant window "
                "has focus, and Ctrl+C remains available in the terminal. "
                "Install 'pynput' and grant macOS keyboard-monitoring "
                "permission for a system-wide hotkey."
            )

        participant_window = ParticipantWindow(
            ui_queue,
            training_intro_queue,
            training_control_queue,
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
                first_section,
                ui_queue,
                training_intro_queue,
                training_control_queue,
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
