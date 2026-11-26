# 香橙派 FAST-LIVO2 建图

`main` 对应已部署的阶段 A。`research/stage-b0` 保存调整启动 IMU 边界的隔离研究版本。每条分支的 `deploy/aarch64/fastlivo_mapping` 都来自该版本板端的安装目录。检出后执行以下命令可在无 ROS 环境的电脑上核对算法源码、传感器配置、标定、固定依赖和 ELF。

```bash
python3 tools/verify_release.py
```

| 目录 | 内容 |
|---|---|
| `src/` | fast_livo、vikit_common、vikit_ros 和专用 Livox ROS 2 驱动，包含项目的时间共享改动 |
| `camera/`、`runtime/`、`tests/` | 海康相机采集、雷达运行管理、归档与重力转换、回归夹具 |
| `configs/` | 正式参数层、相机内参、雷达至相机外参、时间补偿与传感器网络配置 |
| `deps/`、`vendor-src/` | Sophus 头文件及 CMake 导出、板端 Livox SDK 头文件和 ARM64 共享库，以及使用系统 spdlog 的 SDK 源码 |
| `deploy/` | 当前分支的安装 ELF、SHA256、GNU Build ID 和维护源码变动摘要 |
| `records/` | 原始发布身份、冻结标定摘要、历史离线结果和板端地图归档摘要 |

## 在独立目录构建

目标为 Ubuntu 22.04 ARM64 与 ROS 2 Humble。工作树须位于用户可写目录。开发环境需有 colcon、CMake、C++17 编译器、PCL 1.12、OpenCV 4.5、Eigen 3.4、fmt 8、spdlog 1.9、Boost、ROSIDL、ROS 图像传输和 cv_bridge。可用 `rosdep install --from-paths src --ignore-src --rosdistro humble` 检查包清单中的系统依赖，Sophus、专用驱动及 MVS 使用下述固定来源。请从未加载其他 ROS 工作区的 Bash 执行。

```bash
./build.sh dependencies
./build.sh algorithm
./build.sh tests
```

相机组件需要本地海康 MVS 5.1.0 Build20260909 的 Linux aarch64 安装介质，SDK 为 4.8.2.2 Build20260828。厂商外层压缩包 SHA256 为 `9965cd67378a047d5ea500d6257ac9d5ce6e5cffec8cfbe3484e1f7ed5c4fb7e`。从获准使用的 SDK 还原 `deps/mvs/include` 和 `deps/mvs/lib/aarch64`，保留库名软链接。运行 `python3 tools/verify_release.py --with-mvs` 对照板端清单，再运行 `./build.sh camera`。四个阶段也可由 `./build.sh` 一次完成。该 SDK 的 libusb 仅通过相机进程自己的库路径加载。 [MVS 厂商下载](https://www.hikrobotics.com/en/machinevision/service/download/)

ARM64 正式使用仓内的 Livox 库。在其他 CPU 架构上做开发构建时，在单独检出的工作树用 `cmake -S vendor-src/livox_sdk -B build/sdk -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX="$PWD/deps/livox_sdk"`，随后执行 `cmake --build build/sdk --parallel 2` 和 `cmake --install build/sdk`。这样生成的同架构 SDK 与板端共享库哈希不同。SDK 上游参考及修改见[第三方来源](docs/third-party.md)。

## 部署与运行

在新的 ARM64 工作目录先构建依赖与算法以生成本目录的 ROS 安装和类型支持。需要恢复已核对的安装 ELF 时，在该独立目录执行下列命令并加载环境，避免覆盖正在工作的正式树。

```bash
python3 tools/verify_release.py
install -m 0755 deploy/aarch64/fastlivo_mapping install/fast_livo/lib/fast_livo/fastlivo_mapping
cmp deploy/aarch64/fastlivo_mapping install/fast_livo/lib/fast_livo/fastlivo_mapping
source ./env.sh
ldd install/fast_livo/lib/fast_livo/fastlivo_mapping
```

现场采集前，从本机保管的配置设置并导出 `FAST_LIVO2_RADAR_SERIAL`、`FAST_LIVO2_CAMERA_SERIAL`。如需持久化，可用权限 0600 的 `device-identity.local.sh` 保存两个 `export`，开始采集的同一 Bash 中执行 `source ./device-identity.local.sh`。Git 忽略该文件。雷达与相机仍须匹配各自序列号，缺少参数会在运行预检时返回错误。离线回放无需这两个现场参数。

阶段 A 的 `./start.sh`、`./status.sh`、`./view.sh`、`./stop.sh` 与 `./replay.sh <bag目录>` 操作见[板端原版说明](docs/board-operation.md)。板端历史网络拓扑和网口配置仍在传感器配置与运行脚本内，迁移到另一套网口或地址时须同步调整。新构建的相机程序读取本地配置。当前板端发布身份、版本对照与已验证范围见[核对记录](docs/release-verification.md)。
