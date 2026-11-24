# MobilityBridge end-to-end test (headless)

Runs Unity (flat ground, coupled rover, `MobilityBridge` in Effort mode) ↔ `ros_tcp_endpoint`
↔ ros_mobility (`domain:=unity demo:=true`) and prints the acceptance table.

    Tools/MobilityBridgeTest/run_bridge_test.sh <tag> [extra_params.yaml] [run_seconds]

- Builds a throwaway project in `$WORK/project` (default `/tmp/mobility_bridge_test`); never
  opens `mars_1` and never writes this project's Assets. Delete `$WORK/project` when done.
- Needs ~1.2 GB free disk (aborts below `MINFREE_MB`, default 700).
- Needs `~/ROS_unity/ros_ws` (ros_tcp_endpoint) and `~/ros_mobility` built.
- Caps `ulimit -n` at 4096: Unity's mono asserts (`pal_utilities ToFileDescriptor`) with a huge limit.
- Outputs: `<tag>.unity.log`, `<tag>.ros.log`, `<tag>.csv` (rover trace every 0.5 s).
- A segfault line at the very end is Unity being killed by the cleanup; ignore it.
