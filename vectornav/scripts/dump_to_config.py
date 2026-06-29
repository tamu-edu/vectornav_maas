#!/usr/bin/env python3
"""Convert vn_set_antenna_config dump output to a ROS2 params YAML file.

The node's read-only dump includes ROS log prefixes ([INFO], timestamps) and
read-only registers (estimated baseline) that aren't useful for writing back.
This script strips the fluff and emits a clean antenna_offset.yaml that can be
loaded directly by set_antenna_offset.launch.py.

Only writable registers are exported:
  - GpsAntennaOffset          -> gps_antenna_offset
  - GpsCompassBaseline        -> gps_compass_baseline.{position,uncertainty}

The read-only GpsCompassEstimatedBaseline is omitted.

Usage:
    ros2 run vectornav vn_set_antenna_config \
        --ros-args -p serial_port:=/dev/ttyUSB0 -p read_only:=true \
      | python3 scripts/dump_to_config.py > my_antenna_offset.yaml

Then edit the generated file (verify values, set read_only: false) and write it
back:

    ros2 launch vectornav set_antenna_offset.launch.py
"""

import re
import sys
from datetime import datetime


# Map log labels to yaml keys. Only writable registers are included.
LABEL_MAP = {
    "GPS Antenna Offset": "gps_antenna_offset",
    "GPS Compass Baseline Position": "gps_compass_baseline.position",
    "GPS Compass Baseline Uncertainty": "gps_compass_baseline.uncertainty",
}

# Regex for a register value line: "... LABEL: [x, y, z]"
VALUE_RE = re.compile(
    r"(" + "|".join(re.escape(k) for k in LABEL_MAP) + r"):\s*\[([-\d.,\s]+)\]"
)


def parse_dump(lines):
    port = "/dev/ttyUSB0"
    baud = 115200
    model = ""
    firmware = ""
    registers = {}

    for line in lines:
        m = re.search(r"Connecting to : (\S+) @ (\d+) Baud", line)
        if m:
            port, baud = m.group(1), int(m.group(2))
            continue

        m = re.search(r"Model Number: (\S+), Firmware Version: (\S+)", line)
        if m:
            model, firmware = m.group(1).rstrip(","), m.group(2)
            continue

        m = VALUE_RE.search(line)
        if m:
            label = m.group(1)
            values = [float(v.strip()) for v in m.group(2).split(",")]
            registers[LABEL_MAP[label]] = values

    return {
        "port": port,
        "baud": baud,
        "model": model,
        "firmware": firmware,
        "registers": registers,
    }


def fmt_vec(values):
    return "[" + ", ".join(f"{v:.6f}" for v in values) + "]"


def emit_yaml(data):
    lines = []
    now = datetime.now().strftime("%Y-%m-%d")
    lines.append(f"# Generated from vn_set_antenna_config dump on {now}")
    if data["model"]:
        fw = f", Firmware: {data['firmware']}" if data["firmware"] else ""
        lines.append(f"# Device: {data['model']}{fw}")
    lines.append("#")
    lines.append("# NOTE: VectorNav register writes are volatile — values revert on")
    lines.append("# power cycle. To persist to flash, call writeSettings() on the")
    lines.append("# sensor (not yet exposed by this utility). Alternatively, re-run")
    lines.append("# this config on each boot via the launch file.")
    lines.append("vn_set_antenna_config:")
    lines.append("  ros__parameters:")
    lines.append(f'    serial_port: "{data["port"]}"')
    lines.append(f"    serial_baud: {data['baud']}")
    lines.append("    read_only: false")

    regs = data["registers"]
    if "gps_antenna_offset" in regs:
        lines.append(f"    gps_antenna_offset: {fmt_vec(regs['gps_antenna_offset'])}")

    if "gps_compass_baseline.position" in regs:
        lines.append("    gps_compass_baseline:")
        lines.append(
            f"      position: {fmt_vec(regs['gps_compass_baseline.position'])}"
        )
        if "gps_compass_baseline.uncertainty" in regs:
            lines.append(
                f"      uncertainty: {fmt_vec(regs['gps_compass_baseline.uncertainty'])}"
            )

    return "\n".join(lines) + "\n"


def main():
    data = parse_dump(sys.stdin)
    if not data["registers"]:
        print(
            "ERROR: No register values found in input.\n"
            "Make sure to run vn_set_antenna_config with read_only:=true "
            "and pipe its output to this script.",
            file=sys.stderr,
        )
        sys.exit(1)
    sys.stdout.write(emit_yaml(data))


if __name__ == "__main__":
    main()
