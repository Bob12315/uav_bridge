#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Dump MAVLink messages to stdout for quick inspection."""

import argparse
import sys
import time
from collections import Counter

from pymavlink import mavutil


def parse_args():
    parser = argparse.ArgumentParser(description="Dump MAVLink messages.")
    parser.add_argument(
        "--url",
        default="udp:127.0.0.1:14540",
        help="MAVLink connection URL (default: udp:127.0.0.1:14540)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Seconds to capture (default: 10)",
    )
    parser.add_argument(
        "--types",
        default="",
        help="Comma-separated message types to include (e.g. HEARTBEAT,ATTITUDE).",
    )
    parser.add_argument(
        "--no-prints",
        action="store_true",
        help="Do not print each message (only summary).",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    type_filter = {
        t.strip().upper() for t in args.types.split(",") if t.strip()
    }

    master = mavutil.mavlink_connection(args.url)
    master.wait_heartbeat(timeout=10)
    print("heartbeat ok")

    counts = Counter()
    t_end = time.time() + args.duration
    while time.time() < t_end:
        msg = master.recv_match(blocking=True, timeout=1)
        if not msg:
            continue
        msg_type = msg.get_type()
        if type_filter and msg_type.upper() not in type_filter:
            continue
        counts[msg_type] += 1
        if not args.no_prints:
            print(msg)

    print("counts:", dict(counts))
    return 0


if __name__ == "__main__":
    sys.exit(main())
