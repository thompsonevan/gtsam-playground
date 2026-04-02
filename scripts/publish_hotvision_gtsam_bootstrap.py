#!/usr/bin/env python3

"""
One-shot publisher to bootstrap gtsam-node when using rootTableName="HOTVision".

Publishes:
  - HOTVision/input/tag_layout         (string: WPILib AprilTagFieldLayout JSON)
  - HOTVision/input/pose_initial_guess (struct: Pose3d)

Usage:
  python3 scripts/publish_hotvision_gtsam_bootstrap.py \
    --server 127.0.0.1 \
    --layout /path/to/apriltags_field.json \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0
"""

from __future__ import annotations

import argparse
import pathlib
import time

import ntcore
from wpimath.geometry import Pose3d, Translation3d, Rotation3d


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", default="127.0.0.1", help="NT server address (RIO/DS IP)")
    ap.add_argument("--root", default="HOTVision", help="Root table name (default: HOTVision)")
    ap.add_argument("--layout", required=True, help="Path to WPILib AprilTagFieldLayout JSON")

    ap.add_argument("--x", type=float, default=0.0)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--z", type=float, default=0.0)
    ap.add_argument("--roll", type=float, default=0.0, help="radians")
    ap.add_argument("--pitch", type=float, default=0.0, help="radians")
    ap.add_argument("--yaw", type=float, default=0.0, help="radians")
    args = ap.parse_args()

    layout_path = pathlib.Path(args.layout)
    layout_str = layout_path.read_text(encoding="utf-8")

    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("hotvision-gtsam-bootstrap")
    inst.setServer(args.server)

    root = inst.getTable(args.root)
    input_tbl = root.getSubTable("input")

    # Tag layout is a string topic.
    input_tbl.getStringTopic("tag_layout").publish().set(layout_str)

    # Initial pose prior is a Pose3d struct topic.
    pose = Pose3d(
        Translation3d(args.x, args.y, args.z),
        Rotation3d(args.roll, args.pitch, args.yaw),
    )
    input_tbl.getStructTopic("pose_initial_guess", Pose3d).publish().set(pose)

    inst.flush()
    time.sleep(0.1)
    inst.stopClient()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

