#!/bin/bash
# Refresh the reference copy of the Unity-side files from the Unity project.
# The Unity project (~/ROS_unity/unity) is the source of truth; edit there, then run this.
set -e
SRC=${1:-$HOME/ROS_unity/unity}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$HERE/Assets/Scripts/CoupledDiff" "$HERE/Tools/MobilityBridgeTest"
cp "$SRC/Assets/Scripts/CoupledDiff/MobilityBridge.cs" "$HERE/Assets/Scripts/CoupledDiff/"
cp "$SRC/Tools/MobilityBridgeTest/"{BridgeBoot.cs,BridgeTestRunner.cs,run_bridge_test.sh,README.md} "$HERE/Tools/MobilityBridgeTest/"
# RoverDriveController is the team's file: keep only our additions, as a patch.
diff -u "$SRC/Backups/controls_2026-11-24/RoverDriveController.cs" \
        "$SRC/Assets/Scripts/CoupledDiff/RoverDriveController.cs" \
  | sed "1s|.*|--- a/Assets/Scripts/CoupledDiff/RoverDriveController.cs|;2s|.*|+++ b/Assets/Scripts/CoupledDiff/RoverDriveController.cs|" \
  > "$HERE/RoverDriveController.additions.patch" || true
echo "synced from $SRC"
