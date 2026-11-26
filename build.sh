#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "$0")" && pwd -P)"
cd "$root"
phase="${1:-all}"
case "$phase" in all|dependencies|algorithm|camera|tests) ;; *) echo "usage: $0 [all|dependencies|algorithm|camera|tests]" >&2; exit 2;; esac
source "$root/env.sh"
export CMAKE_BUILD_PARALLEL_LEVEL="${FAST_LIVO2_BUILD_JOBS:-3}" MAKEFLAGS="-j${FAST_LIVO2_BUILD_JOBS:-3}"
build_ros() {
  colcon --log-base "$root/build/colcon-log" build --base-paths "$root/src" --executor sequential \
    --build-base "$root/build" --install-base "$root/install" "$@"
}
if [[ "$phase" == all || "$phase" == dependencies ]]; then
  build_ros --packages-select livox_ros_driver2 vikit_common vikit_ros --cmake-args \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble \
    "-DLIVOX_LIDAR_SDK_LIBRARY=$LIVOX_SDK_PREFIX/lib/liblivox_lidar_sdk_shared.so" \
    "-DLIVOX_LIDAR_SDK_INCLUDE_DIR=$LIVOX_SDK_PREFIX/include" "-DSophus_DIR=$Sophus_DIR" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  source "$root/install/local_setup.bash"
fi
if [[ "$phase" == all || "$phase" == camera ]]; then
  cmake -S camera -B build/camera -DCMAKE_BUILD_TYPE=Release "-DMVS_ROOT=$MVS_ROOT" \
    "-DCMAKE_INSTALL_PREFIX=$root/install" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  cmake --build build/camera --parallel "$CMAKE_BUILD_PARALLEL_LEVEL"
  cmake --install build/camera
fi
if [[ "$phase" == all || "$phase" == algorithm || "$phase" == tests ]]; then
  build_ros --packages-select fast_livo --cmake-args -DCMAKE_BUILD_TYPE=Release \
    "-DCMAKE_CXX_FLAGS=-fno-fast-math -ffile-prefix-map=$root=/home/orangepi/fast_livo2" \
    -DBUILD_SHARED_LIBS=OFF "-DSophus_DIR=$Sophus_DIR" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
fi
if [[ "$phase" == tests ]]; then
  cmake --build build/fast_livo --target visual_memory_regression --parallel "$CMAKE_BUILD_PARALLEL_LEVEL"
  "$root/build/fast_livo/visual_memory_regression"
fi
