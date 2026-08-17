#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ||
      ( $# -eq 2 && ${2:-} != "--loop-off" ) ]]; then
  echo "usage: $0 OUTPUT_DIRECTORY [--loop-off]" >&2
  exit 2
fi

OUTPUT_DIRECTORY=$1
LOOP_CLOSURE_ENABLE=true
if [[ ${2:-} == "--loop-off" ]]; then
  LOOP_CLOSURE_ENABLE=false
fi
WORKSPACE=/home/sw/AdaGraph-LIO
PARAMS_FILE=$WORKSPACE/src/hybrid_slam/config/point_lio_urbanv2x.yaml
BAG_DIRECTORY=/run/media/sw/仓库/urbanv2x/large_loop/large_loop_ros2_mcap
POINT_LIO_NODE=$WORKSPACE/install/hybrid_slam/lib/hybrid_slam/point_lio_node

if [[ -e $OUTPUT_DIRECTORY ]]; then
  echo "refusing to overwrite existing output: $OUTPUT_DIRECTORY" >&2
  exit 2
fi
if [[ ! -S /tmp/adagraph_bevplace2.sock ]]; then
  echo "BEVPlace2 socket is unavailable: /tmp/adagraph_bevplace2.sock" >&2
  exit 3
fi

mkdir -p "$OUTPUT_DIRECTORY/map" "$OUTPUT_DIRECTORY/loop_debug" \
  "$OUTPUT_DIRECTORY/ros_log"

# ROS setup scripts probe optional shell variables and are not nounset-safe.
# Keep strict mode for this runner, but suspend it only while sourcing them.
set +u
source /opt/ros/lyrical/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-71}
export ROS_LOG_DIR=$OUTPUT_DIRECTORY/ros_log

NODE_GROUP=
ODOM_GROUP=
PATH_GROUP=
BAG_GROUP=

interrupt_group() {
  local group=${1:-}
  if [[ -n $group ]] && kill -0 -- "-$group" 2>/dev/null; then
    kill -INT -- "-$group" 2>/dev/null || true
  fi
}

terminate_group() {
  local group=${1:-}
  if [[ -n $group ]] && kill -0 -- "-$group" 2>/dev/null; then
    kill -TERM -- "-$group" 2>/dev/null || true
  fi
}

cleanup() {
  interrupt_group "$BAG_GROUP"
  interrupt_group "$NODE_GROUP"
  interrupt_group "$ODOM_GROUP"
  interrupt_group "$PATH_GROUP"
  # ROS 2 bag play can ignore SIGINT when the controlling shell itself is
  # interrupted. Give every owned group a short graceful window, then prevent
  # an aborted experiment from leaking only those exact process groups.
  for _ in {1..30}; do
    if ! { [[ -n $BAG_GROUP ]] && kill -0 -- "-$BAG_GROUP" 2>/dev/null; } &&
       ! { [[ -n $NODE_GROUP ]] && kill -0 -- "-$NODE_GROUP" 2>/dev/null; } &&
       ! { [[ -n $ODOM_GROUP ]] && kill -0 -- "-$ODOM_GROUP" 2>/dev/null; } &&
       ! { [[ -n $PATH_GROUP ]] && kill -0 -- "-$PATH_GROUP" 2>/dev/null; }; then
      return
    fi
    sleep 0.1
  done
  terminate_group "$BAG_GROUP"
  terminate_group "$NODE_GROUP"
  terminate_group "$ODOM_GROUP"
  terminate_group "$PATH_GROUP"
}
trap cleanup EXIT INT TERM

setsid /usr/bin/time -v -o "$OUTPUT_DIRECTORY/resource_usage.txt" \
  "$POINT_LIO_NODE" --ros-args --params-file "$PARAMS_FILE" \
  -p lio.loop_closure.enable:="$LOOP_CLOSURE_ENABLE" \
  -p lio.loop_closure.plugin:=bevplace2 \
  -p lio.loop_closure.debug_directory:="$OUTPUT_DIRECTORY/loop_debug" \
  -p lio.map.save_map:=true \
  -p lio.map.save_map_dir:="$OUTPUT_DIRECTORY/map" \
  -p lio.map.map_name:=large_loop_cartographer_rolling_final.pcd \
  -p lio.map.endpoint_diagnostic_keyframes:=500 \
  >"$OUTPUT_DIRECTORY/point_lio.log" 2>&1 &
NODE_GROUP=$!

sleep 2
if ! kill -0 "$NODE_GROUP" 2>/dev/null; then
  echo "point_lio_node exited during startup" >&2
  wait "$NODE_GROUP" || true
  exit 4
fi

setsid python3 "$WORKSPACE/scripts/record_odom_tum.py" \
  "$OUTPUT_DIRECTORY/estimated.tum" --topic /lio/odom \
  >"$OUTPUT_DIRECTORY/recorder.log" 2>&1 &
ODOM_GROUP=$!
setsid python3 "$WORKSPACE/scripts/record_path_tum.py" \
  "$OUTPUT_DIRECTORY/backend_optimized.tum" \
  --topic /lio/backend/optimized_path \
  >"$OUTPUT_DIRECTORY/backend_recorder.log" 2>&1 &
PATH_GROUP=$!

sleep 1
setsid ros2 bag play "$BAG_DIRECTORY" --rate 1.0 \
  --topics /velodyne_points /imu/data \
  >"$OUTPUT_DIRECTORY/bag_play.log" 2>&1 &
BAG_GROUP=$!
wait "$BAG_GROUP"
BAG_GROUP=

# Let the final backend submission and loop verification drain before asking
# the node to perform its synchronous final trajectory/map save.
sleep 20
interrupt_group "$NODE_GROUP"
wait "$NODE_GROUP"
NODE_GROUP=

interrupt_group "$ODOM_GROUP"
interrupt_group "$PATH_GROUP"
wait "$ODOM_GROUP" || true
wait "$PATH_GROUP" || true
ODOM_GROUP=
PATH_GROUP=

trap - EXIT INT TERM
test -s "$OUTPUT_DIRECTORY/backend_optimized.tum"
test -s "$OUTPUT_DIRECTORY/map/large_loop_cartographer_rolling_final.pcd"
echo "completed: $OUTPUT_DIRECTORY"
