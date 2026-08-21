"""
SHRED participant-window experiment
Version: 5.0

Researcher workflow
-------------------
1. Start the script and type "start" to begin continuous oscillation.
2. The experiment then runs the hard-coded CONDITION_SEQUENCE automatically.
3. Each integer condition code maps to speed/curve/direction:
       0 = 1 1 l, 1 = 1 1 r, 2 = 1 2 l, 3 = 1 2 r, ...
4. The researcher does not enter each trial manually. Type "stop" + Enter
   at any time to request a stop after the current trial.

Participant workflow
--------------------
1. Press Enter when ready.
2. The screen states that the stimulus will show up within 3 seconds.
3. Press Enter as soon as the stimulus is detected.
4. If 3 seconds pass after actual stimulus onset with no detection press,
   the screen warns that the stimulus may have been missed and asks for Enter.
5. Press Left Arrow, Right Arrow, or Down Arrow for "I don't know".

Tkinter runs on the main thread. Serial I/O and the researcher terminal
workflow run on background threads so the participant window remains responsive.

Important macOS detail: after Tk starts, the researcher thread reads terminal
lines with sys.stdin.readline(), not input(). On macOS, input()/readline in a
worker thread can conflict with Tcl/Tk's notifier and abort the process.
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
    1: 0.04,  # slow
    2: 0.02,  # medium
    3: 0.00,  # fast / original P-only behavior
}

DEFAULT_KD = 0.00

# The Teensy firmware must support both names below.
WAVEFORMS = {
    1: "full",
    2: "custom1",
}

# The physical directions on this apparatus were observed to be reversed
# relative to the Teensy's internal L2R/R2L labels. Swap these two values
# back if the physical setup is later corrected.
DIRECTION_TO_TEENSY = {
    "r": "R2L",
    "l": "L2R",
}

DIRECTION_LABEL = {
    "r": "left-to-right",
    "l": "right-to-left",
}

PRINT_BACKGROUND_EVENTS = False
RANDOM_DELAY_MIN_S = 1.0
RANDOM_DELAY_MAX_S = 2.0
MISSED_STIMULUS_WARNING_S = 3.0

# Explicit condition-number table. Keeping this as a dictionary makes the
# numbering visible and easy to audit before data collection.
# Each tuple is: (speed_choice, waveform_choice, direction_key).
CONDITION_MAP = {
    0: (1, 1, "l"),
    1: (1, 1, "r"),
    2: (1, 2, "l"),
    3: (1, 2, "r"),
    4: (2, 1, "l"),
    5: (2, 1, "r"),
    6: (2, 2, "l"),
    7: (2, 2, "r"),
    8: (3, 1, "l"),
    9: (3, 1, "r"),
    10: (3, 2, "l"),
    11: (3, 2, "r"),
}

# Hard-coded experimental order. Edit only this list to change the session.
CONDITION_SEQUENCE = [
    0, 1
]


# ============================================================
# DATA STRUCTURES
# ============================================================


@dataclass(frozen=True)
class TrialConfig:
    trial_id: int
    sequence_index: int
    condition_number: int
    speed_choice: int
    waveform_choice: int
    direction_key: str

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


def create_session_paths() -> tuple[Path, Path]:
    log_dir = Path.cwd() / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    raw_path = log_dir / f"shred_ui_raw_{stamp}.csv"
    results_path = log_dir / f"shred_ui_results_{stamp}.csv"

    return raw_path, results_path


RESULT_FIELDS = [
    "trial_id",
    "sequence_index",
    "condition_number",
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

        self._raw_file: TextIO | None = None
        self._raw_writer: Any | None = None

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
# PARTICIPANT WINDOW
# ============================================================


class ParticipantWindow:
    def __init__(
        self,
        ui_queue: queue.Queue[tuple[str, str]],
        ready_press_queue: queue.Queue[float],
        detection_press_queue: queue.Queue[float],
        response_queue: queue.Queue[tuple[str, float]],
        sensation_active_event: threading.Event,
        shutdown_event: threading.Event,
    ) -> None:
        self.ui_queue = ui_queue
        self.ready_press_queue = ready_press_queue
        self.detection_press_queue = detection_press_queue
        self.response_queue = response_queue
        self.sensation_active_event = sensation_active_event
        self.shutdown_event = shutdown_event

        self.state = "IDLE"
        self.trial_generation = 0

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
            text="Waiting for the researcher to prepare the next trial.",
            font=("Helvetica", 24),
            justify="center",
            wraplength=820,
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
            wraplength=850,
            bg="white",
            fg="black",
        )
        self.hint_label.pack(pady=(20, 70))

        self.root.bind("<Return>", self._on_enter)
        self.root.bind("<KP_Enter>", self._on_enter)
        self.root.bind("<Left>", lambda event: self._on_arrow("l"))
        self.root.bind("<Right>", lambda event: self._on_arrow("r"))
        self.root.bind("<Down>", lambda event: self._on_arrow("down"))

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

    def _on_enter(self, event: tk.Event[Any]) -> str:
        del event

        if self.state == "READY":
            self.ready_press_queue.put(time.monotonic())
            self._show(
                "WAITING",
                "Please wait.\n\nThe stimulus will show up within 3 seconds.",
                "Keep your hands ready on the keyboard.",
            )
            self._schedule_missed_stimulus_warning()

        elif (
            self.state in {"WAITING", "DETECT"}
            and self.sensation_active_event.is_set()
        ):
            self.detection_press_queue.put(time.monotonic())
            self._show(
                "DIRECTION",
                "Which direction did you feel?",
                "←  Left     ↓  I don't know     →  Right",
                focus=True,
            )

        return "break"

    def _on_arrow(self, response_key: str) -> str:
        if self.state != "DIRECTION":
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

    def _schedule_missed_stimulus_warning(self) -> None:
        # The three-second clock starts at the participant's FIRST Enter.
        generation = self.trial_generation
        delay_ms = int(MISSED_STIMULUS_WARNING_S * 1000)

        def show_warning() -> None:
            if (
                generation == self.trial_generation
                and self.state in {"WAITING", "DETECT"}
            ):
                self._show(
                    self.state,
                    "You might have missed the stimulus.\n\nPlease press Enter.",
                    "After pressing Enter, choose ←, →, or ↓.",
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
                        "Press Enter when you are ready.\n\n"
                        "The stimulus will show up within 3 seconds.\n\n"
                        "Press Enter again as soon as you feel the stimulus.",
                        "Then press ← or → for the direction, "
                        "or press ↓ for I don't know.",
                        focus=True,
                    )

                elif command == "DETECT":
                    self._show(
                        "DETECT",
                        "Press Enter as soon as you feel the stimulus.",
                        "",
                        focus=True,
                    )

                elif command == "IDLE":
                    self._show(
                        "IDLE",
                        "Waiting for the researcher to prepare the next trial.",
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
        self.shutdown_event.set()
        self.root.destroy()


# ============================================================
# INPUT AND WORKFLOW HELPERS
# ============================================================


def read_terminal_line(prompt: str) -> str:
    """Read one terminal line without invoking GNU readline/libedit.

    Tk must own the macOS main thread. The researcher workflow therefore runs
    in a worker thread. Built-in input() may invoke readline/libedit, which can
    conflict with Tcl/Tk's event notifier on macOS. A direct TextIO read avoids
    that integration while retaining normal line-based terminal input.
    """
    sys.stdout.write(prompt)
    sys.stdout.flush()

    line = sys.stdin.readline()

    if line == "":
        raise EOFError("Terminal input stream closed")

    return line.rstrip("\r\n")


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


def decode_condition_number(
    condition_number: int,
    trial_id: int,
    sequence_index: int,
) -> TrialConfig:
    try:
        speed_choice, waveform_choice, direction_key = CONDITION_MAP[
            condition_number
        ]
    except KeyError as error:
        valid = ", ".join(str(value) for value in sorted(CONDITION_MAP))
        raise ValueError(
            f"Condition number {condition_number} is invalid; valid codes: {valid}"
        ) from error

    return TrialConfig(
        trial_id=trial_id,
        sequence_index=sequence_index,
        condition_number=condition_number,
        speed_choice=speed_choice,
        waveform_choice=waveform_choice,
        direction_key=direction_key,
    )


def validate_condition_sequence() -> None:
    for index, condition_number in enumerate(CONDITION_SEQUENCE, start=1):
        decode_condition_number(
            condition_number,
            trial_id=index,
            sequence_index=index,
        )


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


def run_trial(
    client: TeensyClient,
    results_path: Path,
    trial: TrialConfig,
    ui_queue: queue.Queue[tuple[str, str]],
    ready_press_queue: queue.Queue[float],
    detection_press_queue: queue.Queue[float],
    response_queue: queue.Queue[tuple[str, float]],
    sensation_active_event: threading.Event,
    shutdown_event: threading.Event,
) -> None:
    drain_queue(ready_press_queue)
    drain_queue(detection_press_queue)
    drain_queue(response_queue)
    sensation_active_event.clear()

    client.command(
        f"SET KD {trial.kd}",
        ("ACK SET KD",),
    )

    print(
        "\nTrial configured:"
        f"\n  trial={trial.trial_id}"
        f"\n  sequence index={trial.sequence_index}/{len(CONDITION_SEQUENCE)}"
        f"\n  condition number={trial.condition_number}"
        f"\n  speed={trial.speed_choice}, KD={trial.kd}"
        f"\n  curve={trial.waveform_choice} ({trial.waveform})"
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

    # Enable the participant's second Enter immediately at actual onset,
    # before the next Tkinter screen refresh. This avoids losing a very
    # fast key press during the small UI-queue polling interval.
    sensation_active_event.set()
    ui_queue.put(("DETECT", ""))

    detection_press_time = float(
        wait_for_queue_item(
            detection_press_queue,
            client,
            shutdown_event,
        )
    )

    reaction_time = detection_press_time - onset_host_time

    response_key, _response_time = wait_for_queue_item(
        response_queue,
        client,
        shutdown_event,
    )

    waveform_end_event, _ = client.wait_for_any_event(
        ("WAVEFORM_ENDED", "WAVEFORM_TIMEOUT"),
        trial_id=trial.trial_id,
        timeout_s=15.0,
    )

    if response_key == "down":
        participant_response = "I don't know"
        correct = False
    else:
        participant_response = DIRECTION_LABEL[response_key]
        correct = response_key == trial.direction_key

    ready_to_onset = onset_host_time - ready_press_time

    print(
        "\nTrial result:"
        f"\n  trial={trial.trial_id}"
        f"\n  reaction time={reaction_time:.3f} s"
        f"\n  participant response={participant_response}"
        f"\n  correct direction={trial.physical_direction}"
        f"\n  correct={'YES' if correct else 'NO'}"
    )

    append_trial_result(
        results_path,
        {
            "trial_id": trial.trial_id,
            "sequence_index": trial.sequence_index,
            "condition_number": trial.condition_number,
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
        },
    )

    ui_queue.put(("IDLE", ""))


def stop_listener_loop(
    stop_requested_event: threading.Event,
    shutdown_event: threading.Event,
) -> None:
    """Listen only for a researcher stop request.

    The experiment itself no longer requires per-trial terminal input.
    Plain sys.stdin.readline() remains macOS/Tk-safe in this worker thread.
    """
    while not shutdown_event.is_set():
        try:
            command_text = read_terminal_line(
                "\nResearcher control: type 'stop' + Enter to stop after the current trial.\n> "
            ).strip().lower()
        except EOFError:
            return

        if command_text == "stop":
            stop_requested_event.set()
            print(
                "\nStop requested. The system will stop safely after "
                "the current trial finishes."
            )
            return

        if command_text:
            print("Only 'stop' is accepted while the automatic sequence is running.")


def experiment_sequence_loop(
    client: TeensyClient,
    results_path: Path,
    ui_queue: queue.Queue[tuple[str, str]],
    ready_press_queue: queue.Queue[float],
    detection_press_queue: queue.Queue[float],
    response_queue: queue.Queue[tuple[str, float]],
    sensation_active_event: threading.Event,
    shutdown_event: threading.Event,
    graceful_stop_event: threading.Event,
    stop_requested_event: threading.Event,
    outcome: dict[str, int],
) -> None:
    print(
        "\nAutomatic condition sequence loaded:"
        f"\n  {CONDITION_SEQUENCE}"
        f"\n  total trials: {len(CONDITION_SEQUENCE)}"
        "\n  mapping: 0=1 1 l, 1=1 1 r, 2=1 2 l, 3=1 2 r, "
        "4=2 1 l, ... 11=3 2 r"
        "\nNo per-trial researcher input is required."
    )

    try:
        for sequence_index, condition_number in enumerate(
            CONDITION_SEQUENCE,
            start=1,
        ):
            if shutdown_event.is_set() or stop_requested_event.is_set():
                break

            trial = decode_condition_number(
                condition_number,
                trial_id=sequence_index,
                sequence_index=sequence_index,
            )

            run_trial(
                client,
                results_path,
                trial,
                ui_queue,
                ready_press_queue,
                detection_press_queue,
                response_queue,
                sensation_active_event,
                shutdown_event,
            )

            if stop_requested_event.is_set():
                break

        client.command(
            "STOP",
            ("ACK STOP",),
        )
        graceful_stop_event.set()
        shutdown_event.set()

        if stop_requested_event.is_set():
            print("\nExperiment stopped by researcher request.")
        else:
            print("\nAll programmed trials are complete.")

        print(
            "Turn OFF motor and magnet power."
            f"\nRaw log: {client.raw_log_path}"
            f"\nResults: {results_path}"
        )

        ui_queue.put(("CLOSE", ""))

    except (
        SerialException,
        RuntimeError,
        TimeoutError,
        OSError,
        ValueError,
    ) as error:
        outcome["code"] = 1
        print(f"\nFAIL: {error}", file=sys.stderr)

        try:
            client.send_line("ESTOP")
            time.sleep(0.2)
        except (SerialException, RuntimeError):
            pass

        ui_queue.put(("ERROR", str(error)))
        time.sleep(1.0)
        ui_queue.put(("CLOSE", ""))


# ============================================================
# MAIN
# ============================================================


def main() -> int:
    validate_condition_sequence()
    raw_path, results_path = create_session_paths()
    initialize_results_log(results_path)

    client = TeensyClient(
        preferred_port=PREFERRED_PORT,
        raw_log_path=raw_path,
    )

    shutdown_event = threading.Event()
    graceful_stop_event = threading.Event()
    stop_requested_event = threading.Event()
    outcome = {"code": 0}
    motion_started = False

    ui_queue: queue.Queue[tuple[str, str]] = queue.Queue()
    ready_press_queue: queue.Queue[float] = queue.Queue()
    detection_press_queue: queue.Queue[float] = queue.Queue()
    response_queue: queue.Queue[tuple[str, float]] = queue.Queue()
    sensation_active_event = threading.Event()

    experiment_thread: threading.Thread | None = None
    stop_listener_thread: threading.Thread | None = None

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

        print(
            "\nSystem is resting."
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
                f"SET KD {DEFAULT_KD}",
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
                f"\nStartup KD={DEFAULT_KD}."
            )
            break

        participant_window = ParticipantWindow(
            ui_queue,
            ready_press_queue,
            detection_press_queue,
            response_queue,
            sensation_active_event,
            shutdown_event,
        )

        experiment_thread = threading.Thread(
            target=experiment_sequence_loop,
            args=(
                client,
                results_path,
                ui_queue,
                ready_press_queue,
                detection_press_queue,
                response_queue,
                sensation_active_event,
                shutdown_event,
                graceful_stop_event,
                stop_requested_event,
                outcome,
            ),
            name="experiment-sequence",
            daemon=True,
        )

        stop_listener_thread = threading.Thread(
            target=stop_listener_loop,
            args=(
                stop_requested_event,
                shutdown_event,
            ),
            name="researcher-stop-listener",
            daemon=True,
        )

        experiment_thread.start()
        stop_listener_thread.start()

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

        client.close()

        print(
            f"\nRaw log: {raw_path}"
            f"\nResults: {results_path}"
        )


if __name__ == "__main__":
    raise SystemExit(main())
