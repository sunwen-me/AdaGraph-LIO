#!/usr/bin/env bash

set -euo pipefail

TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_ROOT="$(cd "${TOOLS_DIR}/.." && pwd)"
ROOT_DIR="${HYBRID_SLAM_ROOT:-${DEFAULT_ROOT}}"
DATASET_ROOT="${DATASET_ROOT:-${ROOT_DIR}/Datasets}"
SEQUENCE="${1:-door_02}"
RATE="${2:-5.0}"
OUTPUT_DIR="${3:-${ROOT_DIR}/log/refactoring_baselines/${SEQUENCE}_point_lio_${RATE}x}"
BAG_DIR="${BAG_DIR:-${DATASET_ROOT}/${SEQUENCE}_ros2}"
GROUND_TRUTH="${GROUND_TRUTH:-${BAG_DIR}/${SEQUENCE}.txt}"

if [[ "${OUTPUT_DIR}" != /* ]]; then
  OUTPUT_DIR="${ROOT_DIR}/${OUTPUT_DIR}"
fi
OUTPUT_DIR="$(realpath -m "${OUTPUT_DIR}")"

NODE_NAME="point_lio_node"
DEFAULT_PARAMS_FILE="${ROOT_DIR}/src/hybrid_slam/config/point_lio_m2dgr.yaml"

PARAMS_FILE="${PARAMS_FILE:-${DEFAULT_PARAMS_FILE}}"
NODE_EXECUTABLE="${ROOT_DIR}/install/hybrid_slam/lib/hybrid_slam/${NODE_NAME}"
NODE_LOG="${OUTPUT_DIR}/point_lio.log"
PRIOR_MAP_FILE="${PRIOR_MAP_FILE:-}"
PRIOR_MAP_REGISTRATION_THREADS="${PRIOR_MAP_REGISTRATION_THREADS:-}"
# Selects the node's startup lifecycle. Empty leaves the params-file value,
# which defaults to mapping. 'localization' additionally requires a prior map.
LIFECYCLE_MODE="${LIFECYCLE_MODE:-}"
ENABLE_BACKEND_LOCALIZATION_FACTOR="${ENABLE_BACKEND_LOCALIZATION_FACTOR:-false}"
BACKEND_DRIVE_MAP_TO_ODOM="${BACKEND_DRIVE_MAP_TO_ODOM:-}"
FRONTEND_HEALTH_ENABLE="${FRONTEND_HEALTH_ENABLE:-}"
FRONTEND_HEALTH_LOST_BAD_FRAMES="${FRONTEND_HEALTH_LOST_BAD_FRAMES:-}"
FRONTEND_HEALTH_RECOVER_GOOD_FRAMES="${FRONTEND_HEALTH_RECOVER_GOOD_FRAMES:-}"
FRONTEND_HEALTH_VERIFICATION_GRACE_FRAMES="${FRONTEND_HEALTH_VERIFICATION_GRACE_FRAMES:-}"
FRONTEND_HEALTH_LIDAR_DROPOUT_LOST_TIMEOUT_S="${FRONTEND_HEALTH_LIDAR_DROPOUT_LOST_TIMEOUT_S:-}"
SUBMAP_RECOVERY_ENABLE="${SUBMAP_RECOVERY_ENABLE:-}"
SUBMAP_RECOVERY_KISS_ENABLE="${SUBMAP_RECOVERY_KISS_ENABLE:-}"
LOOP_CLOSURE_ENABLE="${LOOP_CLOSURE_ENABLE:-}"
LOOP_CLOSURE_PLUGIN="${LOOP_CLOSURE_PLUGIN:-}"
LOOP_CLOSURE_MAX_REGISTRATION_ITERATIONS="${LOOP_CLOSURE_MAX_REGISTRATION_ITERATIONS:-}"
LOOP_CLOSURE_DEBUG_DIRECTORY="${LOOP_CLOSURE_DEBUG_DIRECTORY:-}"
OCCUPANCY_ENABLE="${OCCUPANCY_ENABLE:-}"
OCCUPANCY_SOURCE="${OCCUPANCY_SOURCE:-}"
OCCUPANCY_UPDATE_POLICY="${OCCUPANCY_UPDATE_POLICY:-}"
OCCUPANCY_RESOLUTION="${OCCUPANCY_RESOLUTION:-}"
OCCUPANCY_MAX_EXTENT="${OCCUPANCY_MAX_EXTENT:-}"
OCCUPANCY_REBUILD_MAX_SCRATCH_MEMORY_MB="${OCCUPANCY_REBUILD_MAX_SCRATCH_MEMORY_MB:-}"
OCCUPANCY_REBUILD_MAX_POINTS="${OCCUPANCY_REBUILD_MAX_POINTS:-}"
OCCUPANCY_REBUILD_CANCELLATION_CHECK_INTERVAL="${OCCUPANCY_REBUILD_CANCELLATION_CHECK_INTERVAL:-}"
OCCUPANCY_TOPIC="${OCCUPANCY_TOPIC:-/map}"
CAPTURE_OCCUPANCY="${CAPTURE_OCCUPANCY:-false}"
FAULT_INJECT_ENABLE="${FAULT_INJECT_ENABLE:-false}"
FAULT_LIDAR_DROP="${FAULT_LIDAR_DROP:-false}"
FAULT_WINDOW_START_SECONDS="${FAULT_WINDOW_START_SECONDS:-0}"
FAULT_WINDOW_END_SECONDS="${FAULT_WINDOW_END_SECONDS:-0}"
FAULT_TIMESTAMP_JUMP_SECONDS="${FAULT_TIMESTAMP_JUMP_SECONDS:-0}"
FAULT_CLOCK="${FAULT_CLOCK:-stamp}"
NODE_CPU_AFFINITY="${NODE_CPU_AFFINITY:-}"
BAG_LOOP="${BAG_LOOP:-false}"
BAG_LOOP_COUNT="${BAG_LOOP_COUNT:-2}"
BAG_PLAYBACK_DURATION_SECONDS="${BAG_PLAYBACK_DURATION_SECONDS:-}"
BAG_TOPICS="${BAG_TOPICS:-/velodyne_points /handsfree/imu}"
POST_PLAYBACK_DRAIN_SECONDS="${POST_PLAYBACK_DRAIN_SECONDS:-2}"
NODE_SHUTDOWN_GRACE_SECONDS="${NODE_SHUTDOWN_GRACE_SECONDS:-60}"
NAV2_TF_ENABLE="${NAV2_TF_ENABLE:-}"
NAV2_PUBLISH_MAP_TO_ODOM="${NAV2_PUBLISH_MAP_TO_ODOM:-}"
BACKEND_KEYFRAME_TRANSLATION_M="${BACKEND_KEYFRAME_TRANSLATION_M:-}"
BACKEND_KEYFRAME_ROTATION_RAD="${BACKEND_KEYFRAME_ROTATION_RAD:-}"
BACKEND_KEYFRAME_MAX_FRAMES="${BACKEND_KEYFRAME_MAX_FRAMES:-}"
BACKEND_SUBMAP_MAX_KEYFRAMES="${BACKEND_SUBMAP_MAX_KEYFRAMES:-}"
EVALUATION_ODOMETRY_TRANSLATION_SCALE="${EVALUATION_ODOMETRY_TRANSLATION_SCALE:-}"
EVALUATION_ODOMETRY_YAW_RAD_PER_METER="${EVALUATION_ODOMETRY_YAW_RAD_PER_METER:-}"
SAVE_MAP="${SAVE_MAP:-false}"
SAVE_MAP_DIR="${SAVE_MAP_DIR:-${OUTPUT_DIR}/map}"
MAP_NAME="${MAP_NAME:-map.pcd}"
MAP_DS_SIZE="${MAP_DS_SIZE:-0.3}"
ROS_PARAM_ARGS=()
SUMMARY_OVERRIDE_ARGS=()
SUMMARY_OCCUPANCY_ARGS=()
SUMMARY_PCD_ARGS=()
read -r -a BAG_TOPIC_ARGS <<< "${BAG_TOPICS}"
if [[ "${#BAG_TOPIC_ARGS[@]}" -eq 0 ]]; then
  echo "BAG_TOPICS must contain at least one topic" >&2
  exit 2
fi

if [[ "${FAULT_INJECT_ENABLE}" != "true" &&
      "${FAULT_INJECT_ENABLE}" != "false" ]]; then
  echo "FAULT_INJECT_ENABLE must be true or false" >&2
  exit 2
fi
if [[ "${FAULT_INJECT_ENABLE}" == "true" ]]; then
  if [[ "${FAULT_LIDAR_DROP}" != "true" &&
        "${FAULT_LIDAR_DROP}" != "false" ]]; then
    echo "FAULT_LIDAR_DROP must be true or false" >&2
    exit 2
  fi
  for fault_float in \
    "${FAULT_WINDOW_START_SECONDS}" \
    "${FAULT_WINDOW_END_SECONDS}" \
    "${FAULT_TIMESTAMP_JUMP_SECONDS}"; do
    if [[ ! "${fault_float}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
      echo "fault window and timestamp jump values must be non-negative numbers" >&2
      exit 2
    fi
  done
  if ! awk "BEGIN { exit !(${FAULT_WINDOW_END_SECONDS} > ${FAULT_WINDOW_START_SECONDS}) }"; then
    echo "FAULT_WINDOW_END_SECONDS must be greater than FAULT_WINDOW_START_SECONDS" >&2
    exit 2
  fi
  if [[ "${FAULT_CLOCK}" != "stamp" && "${FAULT_CLOCK}" != "wall" ]]; then
    echo "FAULT_CLOCK must be stamp or wall" >&2
    exit 2
  fi
  ROS_PARAM_ARGS+=(
    -p "imu_topic:=/adagraph_fault/handsfree/imu"
    -p "lidar_topic:=/adagraph_fault/velodyne_points"
  )
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override "fault_inject.enable=true"
    --runtime-override "fault_inject.lidar_drop=${FAULT_LIDAR_DROP}"
    --runtime-override "fault_inject.window_start_seconds=${FAULT_WINDOW_START_SECONDS}"
    --runtime-override "fault_inject.window_end_seconds=${FAULT_WINDOW_END_SECONDS}"
    --runtime-override "fault_inject.timestamp_jump_seconds=${FAULT_TIMESTAMP_JUMP_SECONDS}"
  )
fi

if [[ -n "${PRIOR_MAP_FILE}" ]]; then
  if [[ "${PRIOR_MAP_FILE}" != /* ]] || [[ ! -f "${PRIOR_MAP_FILE}" ]]; then
    echo "PRIOR_MAP_FILE must be an existing absolute file: ${PRIOR_MAP_FILE}" >&2
    exit 1
  fi
  ROS_PARAM_ARGS+=(
    -p "lio.prior_map_localization.enable:=true"
    -p "lio.prior_map_localization.map_file:=${PRIOR_MAP_FILE}"
    -p "lio.kiss_matcher.enable:=true"
    -p "lio.nav2_tf.enable:=true"
    -p "lio.nav2_tf.publish_map_to_odom:=true"
  )
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override "lio.prior_map_localization.enable=true"
    --runtime-override "lio.prior_map_localization.map_file=${PRIOR_MAP_FILE}"
    --runtime-override "lio.kiss_matcher.enable=true"
    --runtime-override "lio.nav2_tf.enable=true"
    --runtime-override "lio.nav2_tf.publish_map_to_odom=true"
  )
fi

if [[ -n "${LIFECYCLE_MODE}" ]]; then
  case "${LIFECYCLE_MODE}" in
    mapping) ;;
    localization)
      if [[ -z "${PRIOR_MAP_FILE}" ]]; then
        echo "LIFECYCLE_MODE=localization requires PRIOR_MAP_FILE" >&2
        exit 2
      fi
      ;;
    *)
      echo "LIFECYCLE_MODE must be mapping or localization" >&2
      exit 2
      ;;
  esac
  ROS_PARAM_ARGS+=( -p "lio.lifecycle.mode:=${LIFECYCLE_MODE}" )
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override "lio.lifecycle.mode=${LIFECYCLE_MODE}"
  )
fi

for nav2_tf_override in \
  "${NAV2_TF_ENABLE}:lio.nav2_tf.enable" \
  "${NAV2_PUBLISH_MAP_TO_ODOM}:lio.nav2_tf.publish_map_to_odom"; do
  nav2_tf_value="${nav2_tf_override%%:*}"
  nav2_tf_parameter="${nav2_tf_override#*:}"
  if [[ -n "${nav2_tf_value}" ]]; then
    case "${nav2_tf_value}" in
      true|false) ;;
      *)
        echo "${nav2_tf_parameter} must be true or false" >&2
        exit 2
        ;;
    esac
    ROS_PARAM_ARGS+=( -p "${nav2_tf_parameter}:=${nav2_tf_value}" )
    SUMMARY_OVERRIDE_ARGS+=(
      --runtime-override "${nav2_tf_parameter}=${nav2_tf_value}"
    )
  fi
done

if [[ -n "${PRIOR_MAP_REGISTRATION_THREADS}" ]]; then
  if [[ -z "${PRIOR_MAP_FILE}" ]]; then
    echo "PRIOR_MAP_REGISTRATION_THREADS requires PRIOR_MAP_FILE" >&2
    exit 2
  fi
  if [[ ! "${PRIOR_MAP_REGISTRATION_THREADS}" =~ ^[1-9][0-9]*$ ]]; then
    echo "PRIOR_MAP_REGISTRATION_THREADS must be a positive integer" >&2
    exit 2
  fi
  ROS_PARAM_ARGS+=(
    -p "lio.prior_map_localization.num_threads:=${PRIOR_MAP_REGISTRATION_THREADS}"
  )
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override
    "lio.prior_map_localization.num_threads=${PRIOR_MAP_REGISTRATION_THREADS}"
  )
fi

if [[ "${ENABLE_BACKEND_LOCALIZATION_FACTOR}" == "true" ]]; then
  ROS_PARAM_ARGS+=(
    -p "lio.backend.use_localization_factor:=true"
    -p "lio.backend.drive_map_to_odom:=true"
  )
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override "lio.backend.use_localization_factor=true"
    --runtime-override "lio.backend.drive_map_to_odom=true"
  )
fi

if [[ -n "${BACKEND_DRIVE_MAP_TO_ODOM}" ]]; then
  case "${BACKEND_DRIVE_MAP_TO_ODOM}" in
    true|false) ;;
    *)
      echo "lio.backend.drive_map_to_odom must be true or false" >&2
      exit 2
      ;;
  esac
  ROS_PARAM_ARGS+=(
    -p "lio.backend.drive_map_to_odom:=${BACKEND_DRIVE_MAP_TO_ODOM}"
  )
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override
    "lio.backend.drive_map_to_odom=${BACKEND_DRIVE_MAP_TO_ODOM}"
  )
fi

if [[ "${SUBMAP_RECOVERY_ENABLE}" == "true" &&
      -z "${SUBMAP_RECOVERY_KISS_ENABLE}" ]]; then
  SUBMAP_RECOVERY_KISS_ENABLE=true
fi

for health_bool_override in \
  "${FRONTEND_HEALTH_ENABLE}:lio.frontend_health.enable" \
  "${SUBMAP_RECOVERY_ENABLE}:lio.submap_recovery.enable" \
  "${SUBMAP_RECOVERY_KISS_ENABLE}:lio.kiss_matcher.enable" \
  "${LOOP_CLOSURE_ENABLE}:lio.loop_closure.enable"; do
  health_value="${health_bool_override%%:*}"
  health_parameter="${health_bool_override#*:}"
  if [[ -n "${health_value}" ]]; then
    case "${health_value}" in
      true|false) ;;
      *)
        echo "${health_parameter} must be true or false" >&2
        exit 2
        ;;
    esac
    ROS_PARAM_ARGS+=( -p "${health_parameter}:=${health_value}" )
    SUMMARY_OVERRIDE_ARGS+=(
      --runtime-override "${health_parameter}=${health_value}"
    )
  fi
done

if [[ -n "${LOOP_CLOSURE_PLUGIN}" ]]; then
  case "${LOOP_CLOSURE_PLUGIN}" in
    scan_context|bevplace2) ;;
    *)
      echo "lio.loop_closure.plugin must be scan_context or bevplace2" >&2
      exit 2
      ;;
  esac
  ROS_PARAM_ARGS+=( -p "lio.loop_closure.plugin:=${LOOP_CLOSURE_PLUGIN}" )
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override "lio.loop_closure.plugin=${LOOP_CLOSURE_PLUGIN}"
  )
fi

if [[ -n "${LOOP_CLOSURE_MAX_REGISTRATION_ITERATIONS}" ]]; then
  if [[ ! "${LOOP_CLOSURE_MAX_REGISTRATION_ITERATIONS}" =~ ^[1-9][0-9]*$ ]]; then
    echo "lio.loop_closure.max_registration_iterations must be a positive integer" >&2
    exit 2
  fi
  ROS_PARAM_ARGS+=(
    -p "lio.loop_closure.max_registration_iterations:=${LOOP_CLOSURE_MAX_REGISTRATION_ITERATIONS}"
  )
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override
    "lio.loop_closure.max_registration_iterations=${LOOP_CLOSURE_MAX_REGISTRATION_ITERATIONS}"
  )
fi

if [[ -n "${LOOP_CLOSURE_DEBUG_DIRECTORY}" ]]; then
  if [[ "${LOOP_CLOSURE_DEBUG_DIRECTORY}" != /* ]]; then
    echo "LOOP_CLOSURE_DEBUG_DIRECTORY must be an absolute path" >&2
    exit 2
  fi
  ROS_PARAM_ARGS+=(
    -p "lio.loop_closure.debug_directory:=${LOOP_CLOSURE_DEBUG_DIRECTORY}"
  )
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override
    "lio.loop_closure.debug_directory=${LOOP_CLOSURE_DEBUG_DIRECTORY}"
  )
fi

if [[ -n "${FRONTEND_HEALTH_LIDAR_DROPOUT_LOST_TIMEOUT_S}" ]]; then
  if ! awk "BEGIN { exit !(${FRONTEND_HEALTH_LIDAR_DROPOUT_LOST_TIMEOUT_S} > 0) }"; then
    echo "lio.frontend_health.lidar_dropout_lost_timeout_s must be positive" >&2
    exit 2
  fi
  ROS_PARAM_ARGS+=(
    -p "lio.frontend_health.lidar_dropout_lost_timeout_s:=${FRONTEND_HEALTH_LIDAR_DROPOUT_LOST_TIMEOUT_S}"
  )
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override "lio.frontend_health.lidar_dropout_lost_timeout_s=${FRONTEND_HEALTH_LIDAR_DROPOUT_LOST_TIMEOUT_S}"
  )
fi

for health_integer_override in \
  "${FRONTEND_HEALTH_LOST_BAD_FRAMES}:lio.frontend_health.lost_bad_frames" \
  "${FRONTEND_HEALTH_RECOVER_GOOD_FRAMES}:lio.frontend_health.recover_good_frames" \
  "${FRONTEND_HEALTH_VERIFICATION_GRACE_FRAMES}:lio.frontend_health.verification_grace_frames"; do
  health_value="${health_integer_override%%:*}"
  health_parameter="${health_integer_override#*:}"
  if [[ -n "${health_value}" ]]; then
    if [[ ! "${health_value}" =~ ^[0-9]+$ ]]; then
      echo "${health_parameter} must be a non-negative integer" >&2
      exit 2
    fi
    if [[ "${health_parameter}" != "lio.frontend_health.verification_grace_frames" &&
          "${health_value}" == "0" ]]; then
      echo "${health_parameter} must be a positive integer" >&2
      exit 2
    fi
    ROS_PARAM_ARGS+=( -p "${health_parameter}:=${health_value}" )
    SUMMARY_OVERRIDE_ARGS+=(
      --runtime-override "${health_parameter}=${health_value}"
    )
  fi
done

if [[ -n "${OCCUPANCY_ENABLE}" ]]; then
  case "${OCCUPANCY_ENABLE}" in
    true|false) ;;
    *)
      echo "OCCUPANCY_ENABLE must be true or false" >&2
      exit 2
      ;;
  esac
  ROS_PARAM_ARGS+=(
    -p "lio.occupancy_grid.enable:=${OCCUPANCY_ENABLE}"
  )
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override "lio.occupancy_grid.enable=${OCCUPANCY_ENABLE}"
  )
fi

if [[ -n "${OCCUPANCY_SOURCE}" ]]; then
  case "${OCCUPANCY_SOURCE}" in
    snapshot|online) ;;
    *)
      echo "OCCUPANCY_SOURCE must be snapshot or online" >&2
      exit 2
      ;;
  esac
  ROS_PARAM_ARGS+=(
    -p "lio.occupancy_grid.source:=${OCCUPANCY_SOURCE}"
  )
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override "lio.occupancy_grid.source=${OCCUPANCY_SOURCE}"
  )
fi

if [[ -n "${OCCUPANCY_UPDATE_POLICY}" ]]; then
  case "${OCCUPANCY_UPDATE_POLICY}" in
    incremental|rebuild|lazy) ;;
    *)
      echo "OCCUPANCY_UPDATE_POLICY must be incremental, rebuild or lazy" >&2
      exit 2
      ;;
  esac
  ROS_PARAM_ARGS+=(
    -p "lio.occupancy_grid.update_policy:=${OCCUPANCY_UPDATE_POLICY}"
  )
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override
    "lio.occupancy_grid.update_policy=${OCCUPANCY_UPDATE_POLICY}"
  )
fi

for occupancy_float_override in \
  "${OCCUPANCY_RESOLUTION}:lio.occupancy_grid.resolution" \
  "${OCCUPANCY_MAX_EXTENT}:lio.occupancy_grid.max_extent"; do
  occupancy_value="${occupancy_float_override%%:*}"
  occupancy_parameter="${occupancy_float_override#*:}"
  if [[ -n "${occupancy_value}" ]]; then
    if [[ ! "${occupancy_value}" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
       ! awk "BEGIN { exit !(${occupancy_value} > 0) }"; then
      echo "${occupancy_parameter} must be a positive number" >&2
      exit 2
    fi
    # ROS CLI infers `100` as an integer, while these parameters are declared
    # as doubles. Preserve a floating-point spelling for integral user input.
    occupancy_ros_value="${occupancy_value}"
    if [[ "${occupancy_ros_value}" != *.* ]]; then
      occupancy_ros_value+=".0"
    fi
    ROS_PARAM_ARGS+=( -p "${occupancy_parameter}:=${occupancy_ros_value}" )
    SUMMARY_OVERRIDE_ARGS+=(
      --runtime-override "${occupancy_parameter}=${occupancy_value}"
    )
  fi
done

for rebuild_override in \
  "${OCCUPANCY_REBUILD_MAX_SCRATCH_MEMORY_MB}:lio.occupancy_grid.rebuild.max_scratch_memory_mb" \
  "${OCCUPANCY_REBUILD_MAX_POINTS}:lio.occupancy_grid.rebuild.max_points" \
  "${OCCUPANCY_REBUILD_CANCELLATION_CHECK_INTERVAL}:lio.occupancy_grid.rebuild.cancellation_check_interval"; do
  rebuild_value="${rebuild_override%%:*}"
  rebuild_parameter="${rebuild_override#*:}"
  if [[ -n "${rebuild_value}" ]]; then
    if [[ ! "${rebuild_value}" =~ ^[1-9][0-9]*$ ]]; then
      echo "${rebuild_parameter} must be a positive integer" >&2
      exit 2
    fi
    ROS_PARAM_ARGS+=( -p "${rebuild_parameter}:=${rebuild_value}" )
    SUMMARY_OVERRIDE_ARGS+=(
      --runtime-override "${rebuild_parameter}=${rebuild_value}"
    )
  fi
done

NODE_COMMAND_PREFIX=()
if [[ -n "${NODE_CPU_AFFINITY}" ]]; then
  if ! taskset -c "${NODE_CPU_AFFINITY}" true 2>/dev/null; then
    echo "NODE_CPU_AFFINITY is not a valid taskset CPU list: ${NODE_CPU_AFFINITY}" >&2
    exit 2
  fi
  NODE_COMMAND_PREFIX=(taskset -c "${NODE_CPU_AFFINITY}")
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override "node.cpu_affinity=${NODE_CPU_AFFINITY}"
  )
fi

if [[ "${BAG_LOOP}" != "true" && "${BAG_LOOP}" != "false" ]]; then
  echo "BAG_LOOP must be true or false" >&2
  exit 2
fi
if [[ ! "${BAG_LOOP_COUNT}" =~ ^[1-9][0-9]*$ ]]; then
  echo "BAG_LOOP_COUNT must be a positive integer" >&2
  exit 2
fi
if [[ -n "${BAG_PLAYBACK_DURATION_SECONDS}" ]]; then
  if [[ ! "${BAG_PLAYBACK_DURATION_SECONDS}" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
     ! awk "BEGIN { exit !(${BAG_PLAYBACK_DURATION_SECONDS} > 0) }"; then
    echo "BAG_PLAYBACK_DURATION_SECONDS must be a positive number" >&2
    exit 2
  fi
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override "bag.playback_duration_seconds=${BAG_PLAYBACK_DURATION_SECONDS}"
  )
fi
if [[ ! "${POST_PLAYBACK_DRAIN_SECONDS}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "POST_PLAYBACK_DRAIN_SECONDS must be a non-negative number" >&2
  exit 2
fi
if [[ ! "${NODE_SHUTDOWN_GRACE_SECONDS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "NODE_SHUTDOWN_GRACE_SECONDS must be a positive integer" >&2
  exit 2
fi
SUMMARY_OVERRIDE_ARGS+=(
  --runtime-override "bag.post_playback_drain_seconds=${POST_PLAYBACK_DRAIN_SECONDS}"
)
if [[ "${BAG_LOOP}" == "true" && -n "${BAG_PLAYBACK_DURATION_SECONDS}" ]]; then
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override "bag.loop_count=${BAG_LOOP_COUNT}"
  )
fi

for backend_float_override in \
  "${BACKEND_KEYFRAME_TRANSLATION_M}:lio.backend.keyframe_translation_m" \
  "${BACKEND_KEYFRAME_ROTATION_RAD}:lio.backend.keyframe_rotation_rad"; do
  backend_value="${backend_float_override%%:*}"
  backend_parameter="${backend_float_override#*:}"
  if [[ -n "${backend_value}" ]]; then
    if [[ ! "${backend_value}" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
       ! awk "BEGIN { exit !(${backend_value} > 0) }"; then
      echo "${backend_parameter} must be a positive number" >&2
      exit 2
    fi
    backend_ros_value="${backend_value}"
    if [[ "${backend_ros_value}" != *.* ]]; then
      backend_ros_value+=".0"
    fi
    ROS_PARAM_ARGS+=( -p "${backend_parameter}:=${backend_ros_value}" )
    SUMMARY_OVERRIDE_ARGS+=(
      --runtime-override "${backend_parameter}=${backend_value}"
    )
  fi
done

if [[ -n "${BACKEND_KEYFRAME_MAX_FRAMES}" ]]; then
  if [[ ! "${BACKEND_KEYFRAME_MAX_FRAMES}" =~ ^[1-9][0-9]*$ ]]; then
    echo "lio.backend.keyframe_max_frames must be a positive integer" >&2
    exit 2
  fi
  ROS_PARAM_ARGS+=(
    -p "lio.backend.keyframe_max_frames:=${BACKEND_KEYFRAME_MAX_FRAMES}"
  )
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override
    "lio.backend.keyframe_max_frames=${BACKEND_KEYFRAME_MAX_FRAMES}"
  )
fi

if [[ -n "${BACKEND_SUBMAP_MAX_KEYFRAMES}" ]]; then
  if [[ ! "${BACKEND_SUBMAP_MAX_KEYFRAMES}" =~ ^[1-9][0-9]*$ ]]; then
    echo "lio.backend.submap_max_keyframes must be a positive integer" >&2
    exit 2
  fi
  ROS_PARAM_ARGS+=(
    -p "lio.backend.submap_max_keyframes:=${BACKEND_SUBMAP_MAX_KEYFRAMES}"
  )
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override
    "lio.backend.submap_max_keyframes=${BACKEND_SUBMAP_MAX_KEYFRAMES}"
  )
fi

for evaluation_drift_override in \
  "${EVALUATION_ODOMETRY_TRANSLATION_SCALE}:lio.evaluation.odometry_translation_scale:positive" \
  "${EVALUATION_ODOMETRY_YAW_RAD_PER_METER}:lio.evaluation.odometry_yaw_rad_per_meter:signed"; do
  IFS=: read -r evaluation_value evaluation_parameter evaluation_kind \
    <<< "${evaluation_drift_override}"
  if [[ -z "${evaluation_value}" ]]; then
    continue
  fi
  if [[ ! "${evaluation_value}" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || \
     { [[ "${evaluation_kind}" == "positive" ]] && \
       ! awk "BEGIN { exit !(${evaluation_value} > 0) }"; }; then
    echo "${evaluation_parameter} must be a ${evaluation_kind} finite number" >&2
    exit 2
  fi
  evaluation_ros_value="${evaluation_value}"
  if [[ "${evaluation_ros_value}" != *.* ]]; then
    evaluation_ros_value+=".0"
  fi
  ROS_PARAM_ARGS+=( -p "${evaluation_parameter}:=${evaluation_ros_value}" )
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override "${evaluation_parameter}=${evaluation_value}"
  )
done

if [[ "${CAPTURE_OCCUPANCY}" == "true" ]]; then
  SUMMARY_OCCUPANCY_ARGS+=(
    --occupancy-summary "${OUTPUT_DIR}/occupancy_summary.json"
  )
fi

if [[ "${SAVE_MAP}" == "true" ]]; then
  if [[ "${SAVE_MAP_DIR}" != /* ]]; then
    echo "SAVE_MAP_DIR must be an absolute path when SAVE_MAP=true" >&2
    exit 2
  fi
  ROS_PARAM_ARGS+=(
    -p "lio.map.save_map:=true"
    -p "lio.map.save_map_dir:=${SAVE_MAP_DIR}"
    -p "lio.map.map_name:=${MAP_NAME}"
    -p "lio.map.ds_size:=${MAP_DS_SIZE}"
  )
  SUMMARY_OVERRIDE_ARGS+=(
    --runtime-override "lio.map.save_map=true"
    --runtime-override "lio.map.save_map_dir=${SAVE_MAP_DIR}"
    --runtime-override "lio.map.map_name=${MAP_NAME}"
    --runtime-override "lio.map.ds_size=${MAP_DS_SIZE}"
  )
  SUMMARY_PCD_ARGS+=(
    --pcd-summary "${OUTPUT_DIR}/pcd_summary.json"
  )
fi

if [[ ! -f "${BAG_DIR}/metadata.yaml" ]]; then
  echo "Dataset not found: ${BAG_DIR}" >&2
  exit 1
fi

if [[ ! -f "${GROUND_TRUTH}" ]]; then
  echo "Ground truth not found: ${GROUND_TRUTH}" >&2
  exit 1
fi

if [[ ! -f "${PARAMS_FILE}" ]]; then
  echo "Parameter file not found: ${PARAMS_FILE}" >&2
  exit 1
fi

if [[ ! -x "${NODE_EXECUTABLE}" ]]; then
  echo "Executable not found; build the workspace first: ${NODE_EXECUTABLE}" >&2
  exit 1
fi

if [[ -d "${OUTPUT_DIR}" ]] && [[ -n "$(find "${OUTPUT_DIR}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo "Refusing to overwrite non-empty output directory: ${OUTPUT_DIR}" >&2
  echo "Choose a new output directory for every regression run." >&2
  exit 1
fi

mkdir -p "${OUTPUT_DIR}"
if [[ -z "${ROS_LOG_DIR:-}" ]]; then
  ROS_LOG_DIR="${OUTPUT_DIR}/ros_log"
fi
mkdir -p "${ROS_LOG_DIR}"
export ROS_LOG_DIR
set +u
source /opt/ros/lyrical/setup.bash
source "${ROOT_DIR}/install/setup.bash"
set -u

lio_pid=""
recorder_pid=""
backend_recorder_pid=""
occupancy_recorder_pid=""
fault_relay_pid=""
bag_pid=""

stop_process_group() {
  local pid="$1"
  if ! kill -0 "${pid}" 2>/dev/null; then
    return
  fi

  kill -INT -- "-${pid}" 2>/dev/null || true
  local shutdown_deadline=$((SECONDS + NODE_SHUTDOWN_GRACE_SECONDS))
  while [[ "${SECONDS}" -lt "${shutdown_deadline}" ]]; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      wait "${pid}" 2>/dev/null || true
      return
    fi
    sleep 0.2
  done

  kill -TERM -- "-${pid}" 2>/dev/null || true
  sleep 1
  kill -KILL -- "-${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  if [[ -n "${recorder_pid}" ]]; then
    kill -INT "${recorder_pid}" 2>/dev/null || true
    wait "${recorder_pid}" 2>/dev/null || true
  fi
  if [[ -n "${backend_recorder_pid}" ]]; then
    kill -INT "${backend_recorder_pid}" 2>/dev/null || true
    wait "${backend_recorder_pid}" 2>/dev/null || true
  fi
  if [[ -n "${occupancy_recorder_pid}" ]]; then
    kill -INT "${occupancy_recorder_pid}" 2>/dev/null || true
    wait "${occupancy_recorder_pid}" 2>/dev/null || true
  fi
  if [[ -n "${lio_pid}" ]]; then
    stop_process_group "${lio_pid}"
  fi
  if [[ -n "${fault_relay_pid}" ]]; then
    stop_process_group "${fault_relay_pid}"
  fi
  if [[ -n "${bag_pid}" ]] && kill -0 "${bag_pid}" 2>/dev/null; then
    kill -INT "${bag_pid}" 2>/dev/null || true
    wait "${bag_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if [[ "${FAULT_INJECT_ENABLE}" == "true" ]]; then
  FAULT_RELAY_LOOP_ARGS=()
  if [[ "${BAG_LOOP}" == "true" && -n "${BAG_PLAYBACK_DURATION_SECONDS}" ]]; then
    FAULT_RELAY_LOOP_ARGS+=(--normalize-loop-time)
  fi
  setsid python3 "${TOOLS_DIR}/fault_inject_sensor_relay.py" \
    --output-lidar /adagraph_fault/velodyne_points \
    --output-imu /adagraph_fault/handsfree/imu \
    --start "${FAULT_WINDOW_START_SECONDS}" \
    --end "${FAULT_WINDOW_END_SECONDS}" \
    $(if [[ "${FAULT_LIDAR_DROP}" == "true" ]]; then echo --drop-lidar; fi) \
    --timestamp-jump "${FAULT_TIMESTAMP_JUMP_SECONDS}" \
    --fault-clock "${FAULT_CLOCK}" \
    "${FAULT_RELAY_LOOP_ARGS[@]}" \
    >"${OUTPUT_DIR}/fault_relay.log" 2>&1 &
  fault_relay_pid="$!"
fi

setsid /usr/bin/time -v -o "${OUTPUT_DIR}/resource_usage.txt" \
  "${NODE_COMMAND_PREFIX[@]}" \
  "${NODE_EXECUTABLE}" \
  --ros-args \
  --params-file "${PARAMS_FILE}" \
  "${ROS_PARAM_ARGS[@]}" \
  >"${NODE_LOG}" 2>&1 &
lio_pid="$!"

for _ in $(seq 1 50); do
  if ros2 topic list 2>/dev/null | grep -qx "/lio/odom"; then
    break
  fi
  sleep 0.2
done

if ! ros2 topic list 2>/dev/null | grep -qx "/lio/odom"; then
  echo "Point-LIO did not expose /lio/odom" >&2
  exit 1
fi

if [[ "${CAPTURE_OCCUPANCY}" == "true" ]]; then
  python3 "${TOOLS_DIR}/record_occupancy_summary.py" \
    "${OUTPUT_DIR}/occupancy_summary.json" \
    "${OCCUPANCY_TOPIC}" \
    >"${OUTPUT_DIR}/occupancy_recorder.log" 2>&1 &
  occupancy_recorder_pid="$!"
fi

python3 "${TOOLS_DIR}/record_odom_tum.py" \
  "${OUTPUT_DIR}/estimated.tum" \
  >"${OUTPUT_DIR}/recorder.log" 2>&1 &
recorder_pid="$!"
python3 "${TOOLS_DIR}/record_path_tum.py" \
  "${OUTPUT_DIR}/backend_optimized.tum" \
  >"${OUTPUT_DIR}/backend_recorder.log" 2>&1 &
backend_recorder_pid="$!"
sleep 0.5

start_ns="$(date +%s%N)"
bag_play_args=(
  "${BAG_DIR}"
  --rate "${RATE}" \
  --topics "${BAG_TOPIC_ARGS[@]}" \
  --disable-keyboard-controls \
  --progress-bar-update-rate 0 \
)
if [[ "${BAG_LOOP}" == "true" && -z "${BAG_PLAYBACK_DURATION_SECONDS}" ]]; then
  bag_play_args+=(--loop)
fi
if [[ "${BAG_LOOP}" != "true" && -n "${BAG_PLAYBACK_DURATION_SECONDS}" ]]; then
  bag_play_args+=(--playback-duration "${BAG_PLAYBACK_DURATION_SECONDS}")
fi
bag_pass_count=1
if [[ "${BAG_LOOP}" == "true" && -n "${BAG_PLAYBACK_DURATION_SECONDS}" ]]; then
  # rosbag2 versions differ in how --loop and --playback-duration interact.
  # Explicit single-pass processes keep the fault window and map lifecycle
  # deterministic for recovery evaluation.
  bag_pass_count="${BAG_LOOP_COUNT}"
  bag_play_args=("${BAG_DIR}" --rate "${RATE}" \
    --topics "${BAG_TOPIC_ARGS[@]}" \
    --disable-keyboard-controls --progress-bar-update-rate 0 \
    --playback-duration "${BAG_PLAYBACK_DURATION_SECONDS}")
fi

play_bag_once() {
  local loop_index="$1"
  printf 'Starting bag pass %s/%s\n' "$loop_index" "${bag_pass_count}" \
    >>"${OUTPUT_DIR}/bag_play.log"
  ros2 bag play "${bag_play_args[@]}" \
    >>"${OUTPUT_DIR}/bag_play.log" 2>&1 &
  bag_pid="$!"
  while kill -0 "${bag_pid}" 2>/dev/null; do
    if ! kill -0 "${lio_pid}" 2>/dev/null; then
      echo "Point-LIO node exited before bag playback completed" >&2
      kill -TERM "${bag_pid}" 2>/dev/null || true
      wait "${bag_pid}" 2>/dev/null || true
      bag_pid=""
      exit 1
    fi
    sleep 0.5
  done
  if ! wait "${bag_pid}"; then
    echo "ros2 bag playback failed" >&2
    bag_pid=""
    exit 1
  fi
  bag_pid=""
}

for loop_index in $(seq 1 "${bag_pass_count}"); do
  play_bag_once "$loop_index"
done
end_ns="$(date +%s%N)"

sleep "${POST_PLAYBACK_DRAIN_SECONDS}"
cleanup
trap - EXIT

python3 -c \
  "print(f'{(${end_ns} - ${start_ns}) / 1e9:.6f}')" \
  >"${OUTPUT_DIR}/wall_time_seconds.txt"

if [[ "${SAVE_MAP}" == "true" ]]; then
  map_pcd_paths=()
  while IFS= read -r -d '' map_path; do
    map_pcd_paths+=("${map_path}")
  done < <(find "${SAVE_MAP_DIR}" -type f -name '*.pcd' -print0 2>/dev/null | sort -z)
  if [[ "${#map_pcd_paths[@]}" -gt 0 ]]; then
    python3 "${TOOLS_DIR}/inspect_pcd.py" \
      "${map_pcd_paths[@]}" \
      --output "${OUTPUT_DIR}/pcd_summary.json"
  else
    python3 - "${OUTPUT_DIR}/pcd_summary.json" <<'PY'
import json
import sys
from pathlib import Path

output = Path(sys.argv[1])
output.write_text(
    json.dumps({"schema_version": 1, "files": []}, indent=2) + "\n",
    encoding="utf-8",
)
PY
  fi
fi

evo_status="not_run"
if command -v evo_ape >/dev/null 2>&1; then
  if evo_ape tum \
      "${GROUND_TRUTH}" \
      "${OUTPUT_DIR}/estimated.tum" \
      -a \
      --save_results "${OUTPUT_DIR}/ape.zip" \
      >"${OUTPUT_DIR}/ape.txt" 2>&1; then
    evo_status="passed"
  else
    evo_status="failed"
  fi
  if [[ -s "${OUTPUT_DIR}/backend_optimized.tum" ]]; then
    if ! evo_ape tum \
        "${GROUND_TRUTH}" \
        "${OUTPUT_DIR}/backend_optimized.tum" \
        -a \
        --save_results "${OUTPUT_DIR}/backend_ape.zip" \
        >"${OUTPUT_DIR}/backend_ape.txt" 2>&1; then
      echo "backend evo_ape failed; inspect ${OUTPUT_DIR}/backend_ape.txt" >&2
      exit 1
    fi
  else
    echo "backend optimized trajectory was not recorded" >&2
    exit 1
  fi
fi
printf '%s\n' "${evo_status}" >"${OUTPUT_DIR}/ape_status.txt"

git_commit="$(git -C "${ROOT_DIR}" rev-parse HEAD)"
git_dirty_args=()
if ! git -C "${ROOT_DIR}" diff --quiet || \
   ! git -C "${ROOT_DIR}" diff --cached --quiet || \
   [[ -n "$(git -C "${ROOT_DIR}" ls-files --others --exclude-standard)" ]]; then
  git_dirty_args+=(--git-dirty)
fi

python3 "${TOOLS_DIR}/summarize_tum.py" \
  --trajectory "${OUTPUT_DIR}/estimated.tum" \
  --backend-trajectory "${OUTPUT_DIR}/backend_optimized.tum" \
  --ground-truth "${GROUND_TRUTH}" \
  --ape "${OUTPUT_DIR}/ape.txt" \
  --backend-ape "${OUTPUT_DIR}/backend_ape.txt" \
  --resource-usage "${OUTPUT_DIR}/resource_usage.txt" \
  --node-log "${NODE_LOG}" \
  --params "${PARAMS_FILE}" \
  --frontend point_lio \
  --sequence "${SEQUENCE}" \
  --rate "${RATE}" \
  --wall-seconds "$(cat "${OUTPUT_DIR}/wall_time_seconds.txt")" \
  --git-commit "${git_commit}" \
  "${SUMMARY_OVERRIDE_ARGS[@]}" \
  "${SUMMARY_OCCUPANCY_ARGS[@]}" \
  "${SUMMARY_PCD_ARGS[@]}" \
  "${git_dirty_args[@]}" \
  --output "${OUTPUT_DIR}/summary.json"

echo "sequence=${SEQUENCE}"
echo "frontend=point_lio"
echo "rate=${RATE}"
echo "poses=$(wc -l < "${OUTPUT_DIR}/estimated.tum")"
echo "wall_seconds=$(cat "${OUTPUT_DIR}/wall_time_seconds.txt")"
echo "ape_status=${evo_status}"
echo "summary=${OUTPUT_DIR}/summary.json"

if [[ "${evo_status}" == "failed" ]]; then
  echo "evo_ape failed; inspect ${OUTPUT_DIR}/ape.txt" >&2
  exit 1
fi
