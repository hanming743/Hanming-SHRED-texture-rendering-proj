"""
SHRED Magnet PWM Calibration Terminal

Run this while shred_magnet_pwm_calibration.ino is loaded on the Teensy.

Commands:
    0-255      set magnet PWM
    g <value>  log the gauss-meter reading at the current PWM
    off        set PWM to 0
    status     read current PWM
    quit       turn magnet off and exit
"""

from __future__ import annotations

import csv
import glob
import time
from datetime import datetime
from pathlib import Path

import serial

BAUD = 115200
PREFERRED_PORT = None


def find_teensy_port() -> str:
    if PREFERRED_PORT:
        return PREFERRED_PORT

    candidates = sorted(
        glob.glob("/dev/cu.usbmodem*")
        + glob.glob("/dev/tty.usbmodem*")
    )

    if not candidates:
        raise RuntimeError(
            "No Teensy serial port found. "
            "Check USB or set PREFERRED_PORT manually."
        )

    return candidates[0]


def send_and_read(ser: serial.Serial, command: str) -> list[str]:
    ser.write((command + "\n").encode("utf-8"))
    ser.flush()
    time.sleep(0.08)

    lines = []
    while ser.in_waiting:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if line:
            lines.append(line)
    return lines


def main() -> None:
    port = find_teensy_port()
    print(f"Connecting to {port} at {BAUD} baud...")

    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(1.0)

    while ser.in_waiting:
        print(ser.readline().decode("utf-8", errors="replace").rstrip())

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = Path.cwd() / f"magnet_pwm_gauss_{stamp}.csv"

    current_pwm = 0

    print("\nCommands:")
    print("  0-255      set magnet PWM")
    print("  g <value>  log gauss reading at current PWM")
    print("  off        magnet PWM = 0")
    print("  status     show current PWM")
    print("  quit       turn magnet off and exit")
    print(f"\nMeasurements will be saved to: {csv_path}")

    try:
        while True:
            entry = input("\nPWM / command > ").strip()

            if not entry:
                continue

            if entry.lower() in {"quit", "exit", "q"}:
                for line in send_and_read(ser, "off"):
                    print(line)
                print("Magnet turned off.")
                break

            if entry.lower().startswith("g "):
                parts = entry.split(maxsplit=1)

                try:
                    gauss_value = float(parts[1])
                except (IndexError, ValueError):
                    print("Use: g <gauss_value>   Example: g 123.4")
                    continue

                file_exists = csv_path.exists()

                with csv_path.open("a", newline="", encoding="utf-8") as f:
                    writer = csv.writer(f)

                    if not file_exists:
                        writer.writerow(
                            ["timestamp_iso", "pwm", "gauss_reading"]
                        )

                    writer.writerow(
                        [
                            datetime.now().astimezone().isoformat(
                                timespec="milliseconds"
                            ),
                            current_pwm,
                            gauss_value,
                        ]
                    )

                print(
                    f"Logged: PWM={current_pwm}, gauss={gauss_value}"
                )
                continue

            lines = send_and_read(ser, entry)

            for line in lines:
                print(line)

                if line.startswith("MAG_PWM="):
                    try:
                        current_pwm = int(line.split("=", 1)[1])
                    except ValueError:
                        pass

    finally:
        try:
            ser.write(b"off\n")
            ser.flush()
            time.sleep(0.05)
        except Exception:
            pass

        ser.close()
        print(f"\nCSV: {csv_path}")


if __name__ == "__main__":
    main()
