#!/usr/bin/env bash
# Source this file from bash. Only the system Humble installation and this tree are used.
FAST_LIVO2_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)" || return 1
export FAST_LIVO2_ROOT
_fl_env() {
  local p
  local -a paths
  IFS=: read -r -a paths <<< "${AMENT_PREFIX_PATH:-}"
  for p in "${paths[@]}"; do
    case "$p" in ""|/opt/ros/humble|"$FAST_LIVO2_ROOT"/install/*) ;; *)
      echo "已有其它 ROS overlay，请从干净 shell 启动：$p" >&2; return 1 ;;
    esac
  done
  source /opt/ros/humble/setup.bash || return 1
  if [[ -f "$FAST_LIVO2_ROOT/install/local_setup.bash" ]]; then
    source "$FAST_LIVO2_ROOT/install/local_setup.bash" || return 1
  fi
  export Sophus_DIR="$FAST_LIVO2_ROOT/deps/sophus/share/sophus/cmake"
  export MVS_ROOT="$FAST_LIVO2_ROOT/deps/mvs" LIVOX_SDK_PREFIX="$FAST_LIVO2_ROOT/deps/livox_sdk"
  # MVS bundles an older libusb with the system SONAME. Only the camera process may load it.
  export LD_LIBRARY_PATH="$LIVOX_SDK_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export ROS_DOMAIN_ID=215 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE="$FAST_LIVO2_ROOT/configs/sensors/fastdds_camera.xml"
  export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 PYTHONUNBUFFERED=1
}
_fl_env
_fl_result=$?
unset -f _fl_env
return "$_fl_result"
