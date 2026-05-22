#!/usr/bin/env python3
"""
Trajectory log capture — listens on the GRiSP serial port and records
trajectory log rows to a CSV file.

Protocol (emitted by csv_logger.erl in main_loop):
    ===LOG_START_TRAJ===              start sentinel — emitted on F+B press (idle→running)
    TLOG_HEADER,col1,col2,...         CSV header line
    TLOG,val1,val2,...                CSV data rows (~30 Hz, rate-limited in csv_logger)
    ===LOG_END_TRAJ===                end sentinel — emitted when trajectory finishes

Lines without the TLOG prefix between the sentinels (other shell chatter,
[Robot] messages, etc.) are ignored. Column headers are read dynamically
from the TLOG_HEADER line, so no changes are needed here when columns change.

Note: the CONFIG footer is written to the SD card (LOGS/traj_log.csv) by the
robot, not captured over serial. Pull it from the SD card after the run.

Usage:
    pip install pyserial
    python3 traj_log.py --port /dev/tty.usbserial-XXXX
    python3 traj_log.py --port COM5 --out my_run.csv

Press Ctrl+C at any time to abort and save what's been captured so far.
"""

import argparse
import csv
import re
import sys
import time

# Strips the Erlang shell prompt like "(balancing_robot@my_grisp_board)1> "
# that gets prepended to io:format output when running inside the shell.
PROMPT_RE = re.compile(r'^\s*\([^)\s]+\)\d+>\s*')

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial", file=sys.stderr)
    sys.exit(2)


def pick_port():
    """List available serial ports and prompt user for a selection."""
    ports = list(serial.tools.list_ports.comports())
    print("\nAvailable Serial Ports:")
    for i, p in enumerate(ports):
        desc = f" ({p.description})" if p.description and p.description != "n/a" else ""
        print(f"  [{i}] {p.device}{desc}")

    if not ports:
        print("No devices found. Check your connection.")
        sys.exit(1)

    while True:
        choice = input("\nSelect port index: ").strip()
        try:
            idx = int(choice)
            if 0 <= idx < len(ports):
                return ports[idx].device
        except ValueError:
            pass
        print(f"Enter a number between 0 and {len(ports)-1}.")


# ─── Sentinels & prefixes (must match the Erlang side) ─────────────────────
START_SENT = "===LOG_START_TRAJ==="
END_SENT   = "===LOG_END_TRAJ==="
HEADER_PFX = "TLOG_HEADER,"
ROW_PFX    = "TLOG,"



def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None,
                    help="Serial port (skip to choose interactively). "
                         "e.g. /dev/tty.usbserial-XXXX (macOS), "
                         "/dev/ttyUSB0 (Linux), COM5 (Windows)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out",  default="traj_log.csv")
    args = ap.parse_args()

    port = args.port if args.port else pick_port()

    print(f"[traj_log] Opening {port} @ {args.baud} baud…", flush=True)
    ser = serial.Serial(port, args.baud, timeout=1.0)

    print(f'[traj_log] Waiting for "{START_SENT}". Trigger the trajectory on the robot.', flush=True)
    print("[traj_log] (Ctrl+C to abort.)", flush=True)

    capturing = False
    header    = None
    rows      = []
    t_open    = time.time()

    try:
        while True:
            raw = ser.readline()
            if not raw:
                # Show occasional heartbeat while waiting
                if not capturing and time.time() - t_open > 10 and int(time.time()) % 10 == 0:
                    print(f"[traj_log] …still waiting (no serial data yet?)", flush=True)
                    t_open = time.time()
                continue

            try:
                line = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                continue

            # Strip Erlang shell prompt prefix if present.
            line = PROMPT_RE.sub("", line)

            if not capturing:
                if line == START_SENT:
                    capturing = True
                    print("[traj_log] >>> START detected. Capturing…", flush=True)
                # else: ignore pre-start chatter
                continue

            # Now in capturing mode
            if line == END_SENT:
                print("[traj_log] >>> END detected. Closing capture.", flush=True)
                break
            elif line.startswith(HEADER_PFX):
                header = line[len(HEADER_PFX):].split(",")
                print(f"[traj_log] Header: {header}", flush=True)
            elif line.startswith(ROW_PFX):
                fields = line[len(ROW_PFX):].split(",")
                rows.append(fields)
                # Live print: pair with header if we have one, else raw fields.
                if header is not None and len(fields) == len(header):
                    pairs = " ".join(f"{h}={v}" for h, v in zip(header, fields))
                    print(f"[{len(rows):04d}] {pairs}", flush=True)
                else:
                    print(f"[{len(rows):04d}] {','.join(fields)}", flush=True)
            # else: filter out non-TLOG lines (other Erlang prints)
    except KeyboardInterrupt:
        print("\n[traj_log] Interrupted — saving partial capture.", flush=True)
    finally:
        ser.close()

    if header is None and not rows:
        print("[traj_log] No data captured. Nothing written.", flush=True)
        sys.exit(1)

    # If we captured rows but no header (unlikely), fabricate one with positional names.
    if header is None:
        n = max(len(r) for r in rows)
        header = [f"col{i}" for i in range(n)]

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow(r)

    print(f"[traj_log] Wrote {len(rows)} rows to {args.out}", flush=True)


if __name__ == "__main__":
    main()
