#!/bin/bash
# Headless end-to-end test: Unity (flat ground, coupled rover, MobilityBridge) <-> ROS-TCP
# endpoint <-> ros_mobility (domain:=unity demo:=true). Throwaway project; never opens mars_1,
# never touches this project's Assets.
# usage: Tools/MobilityBridgeTest/run_bridge_test.sh <tag> [extra_params.yaml] [run_seconds]
# Results in $WORK/<tag>.{unity.log,ros.log,csv}
# (no set -u: ROS setup scripts use unset variables)
TAG=${1:?tag}; EXTRA=${2:-}; RUN=${3:-260}; FPS=${FPS:-0}   # FPS=30 caps Unity's frame rate (editor-like delay)
HERE=$(cd "$(dirname "$0")" && pwd); PROJ=$(cd "$HERE/../.." && pwd)
# Run from the ros_mobility reference copy: fall back to the real Unity project.
[ -d "$PROJ/Assets" ] || PROJ=${UNITY_PROJECT:-$HOME/ROS_unity/unity}
UNITY=/home/apratim/Unity/Hub/Editor/6000.1.17f1/Editor/Unity
WORK=${WORK:-/tmp/mobility_bridge_test}; T=$WORK/project; mkdir -p $WORK
MINFREE_MB=${MINFREE_MB:-700}
free_mb() { df --output=avail -BM "$WORK" | tail -1 | tr -dc 0-9; }
[ "$(free_mb)" -lt 1200 ] && { echo "only $(free_mb) MB free, refusing"; exit 1; }
if [ ! -d $T ]; then
  mkdir -p $T/Assets/PreFabs $T/Assets/Scripts $T/Assets/Test/Editor $T/Library
  cp -r "$PROJ/ProjectSettings" "$PROJ/Packages" $T/
  cp -al "$PROJ/Library/PackageCache" $T/Library/
  for d in urdf_full_withdifferential Materials Settings Editor; do cp -r "$PROJ/Assets/$d" "$PROJ/Assets/$d.meta" $T/Assets/; done
  cp "$PROJ"/Assets/PreFabs/urdf_full_withdifferential_coupled.prefab* $T/Assets/PreFabs/
fi
rm -rf $T/Assets/Scripts/CoupledDiff && cp -r "$PROJ/Assets/Scripts/CoupledDiff" "$PROJ/Assets/Scripts/CoupledDiff.meta" $T/Assets/Scripts/
cp "$HERE/BridgeTestRunner.cs" $T/Assets/Test/ && cp "$HERE/BridgeBoot.cs" $T/Assets/Test/Editor/

ulimit -n 4096   # Unity's mono asserts (pal_utilities ToFileDescriptor) with a huge fd limit
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-79}
source /opt/ros/jazzy/setup.bash
source /home/apratim/ROS_unity/ros_ws/install/setup.bash
source /home/apratim/ros_mobility/install/setup.bash
ULOG=$WORK/$TAG.unity.log; RLOG=$WORK/$TAG.ros.log; rm -f $ULOG $RLOG
# Never collide with a live session: the port must be free.
if ss -ltn 2>/dev/null | grep -q ":10000 "; then
  echo "port 10000 is in use (a ros_mobility/Unity session?); stop it first"; exit 1
fi
# Start the endpoint executable directly (not via `ros2 run`), so $! is the real process
# and cleanup cannot leave it holding port 10000.
"$(ros2 pkg prefix ros_tcp_endpoint)/lib/ros_tcp_endpoint/default_server_endpoint" \
  --ros-args -p ROS_IP:=127.0.0.1 > $WORK/$TAG.endpoint.log 2>&1 &
EP=$!
"$UNITY" -batchmode -nographics -disable-assembly-updater -projectPath $T -executeMethod BridgeBoot.Run \
  -logFile $ULOG -runSeconds $RUN -targetFps $FPS -traceOut $WORK/$TAG.csv > /dev/null 2>&1 &
UP=$!
cleanup() { kill $UP 2>/dev/null; [ "$LP" != 0 ] && kill -INT $LP 2>/dev/null; sleep 2; kill $EP 2>/dev/null; kill -9 $UP $EP 2>/dev/null; }
LP=0; trap cleanup EXIT
echo "waiting for Unity (import can take minutes)..."
for i in $(seq 1 600); do
  grep -q "\[MobilityBridge\] ready" $ULOG 2>/dev/null && break
  kill -0 $UP 2>/dev/null || { echo "Unity exited early"; tail -30 $ULOG; exit 1; }
  [ "$(free_mb)" -lt $MINFREE_MB ] && { echo "disk below ${MINFREE_MB} MB, aborting"; exit 1; }
  sleep 1
done
grep -q "\[MobilityBridge\] ready" $ULOG || { echo "bridge never came up"; exit 1; }
echo "Unity up after ${i}s; starting ros_mobility demo"
ARGS="domain:=unity start_endpoint:=false demo:=true"
[ -n "$EXTRA" ] && ARGS="$ARGS extra_params:=$EXTRA"
timeout 200 ros2 launch mobility_bringup mobility.launch.py $ARGS > $RLOG 2>&1 &
LP=$!
while kill -0 $LP 2>/dev/null; do
  [ "$(free_mb)" -lt $MINFREE_MB ] && { echo "disk below ${MINFREE_MB} MB, aborting"; exit 1; }
  sleep 2
done
sleep 4   # let the bridge notice the stream stopped and hand back
grep -E "\[MobilityBridge\]|\[BridgeTest\]|\[RoverDrive\]" $ULOG | grep -v "^\s*$" | head -20
sed -n '/segment/,/RESULT/p' $RLOG | sed 's/^\[scripted_drive-[0-9]*\] //'
echo "logs: $ULOG $RLOG $WORK/$TAG.csv  (rm -rf $T to free the throwaway project)"
