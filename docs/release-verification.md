# 建图版本核对

## 2026-11-27 · B1

B1 基于 G3 的 B0 固定提交 `fc468aa9cec0d537f6d2d49ad27057d097434152`。目标板原生 Release，保留 strict-math 和 4 线程 OpenMP。构造函数、ROS 参数加载与 `Preprocess::set` 共用距离设置函数。Mid-360S 的三维距离平方与 0.25 m² 比较，恰为 0.5 m 的点保留。可选特征提取中的水平距离按米比较，整线近点有界退出。

板端对生产处理函数送入 Livox CustomMsg，核验构造默认值、二次设值、无效输入、零阈值、整线近点及正式配置 3 取 1 下的 0.499/0.500/0.501 m 和多轴点。可用 `cmake --build build/fast_livo --target blind_regression` 构建相同夹具，加载 `env.sh` 后执行 `build/fast_livo/blind_regression --ros-args --params-file <正式六层合并后的resolved.yaml>`。

60.167 s 手持包一次 1× 回放到 EOF，三路收发差均为零，81 项运行参数一致。由原包复算的 456 条初始化 IMU 与本次重力事件唯一吻合。地图为 `map_gravity`，几何 1,132,757 点、彩色 203,173 点、轨迹 576 个位姿。算法、播放器、参数探针、归档器退出码均为 0。实际部署沿用板端硬件识别、标定和 NVMe 路径。A 与 B0 的历史标签和隔离 B0 均保留。首次有人值守手推式移动实采已另行归档。独立真值、绝对曝光同步和最终平台长时验证待开展。[Baseline 01 实验报告](experiments/2026-11-27/first_corridor_test/FAST-LIVO2_B1_首次走廊移动建图实验技术报告_20261127.md)。详见[发布身份和制品校验值](../deploy/release.json)。

## 2026-11-26 · G3 历史基线

通过已固定主机密钥的只读 SSH 读取两处已有程序、源码与记录。A 位于 `/home/orangepi/fast_livo2`，B0 位于 `/home/orangepi/stage_b0_20261123/candidate`。本轮的构建和回归均在工作站执行，板端原部署和会话继续保留。

| 身份 | 正式 A | 隔离 B0 |
|---|---|---|
| 本仓分支 | main | research/stage-b0 |
| G2 原始依据 | Git 源提交 `a9727f89e348faa218410bce9e834992395d54c2` | 基于同一提交；补丁 SHA256 `57ebeea36023239e816860f249ebdd12df983fa30c218fd0492432adb779d594` |
| 本轮安装 ELF SHA256 | `b66a92cd81de2ebc9428c4483908a97e1e1bfd207802a4cca8ee4829e683d49e` | `e9f7eaeb7260fe13304e1b846426b6a21dc91776d6128f991e6caeacab481469` |
| 本轮 Build ID | `14f4b686dcbd40ba5ba79bd24d0e39a2ce471cf2` | `85e725b4b672db14eaa254e07e6c829bde4cbdf6` |
| 本轮 build/ 下的 ELF SHA256 | `12f89544a78fdc7b03823ec26f32795fd89a3614beb9dcf7bc920bd20ac25d2c` | `9791fa3f693b30e3d7bb94b91aea807c829b90e068c2addcffa2bbc0b385121c` |

两处安装 ELF 均与 G2 历史安装摘要逐字节相符。A 的板端维护清单本身 SHA256 为 `e6c985eb6092604ce7aff2ebd82f706c8b91d9ce1f9cd4215c571a2c89733b11`。逐文件对照，A 的 245/245 项与 G2 原始字节一致。B0 现场候选包含其中 235 项，233 项相同；两处差异为 `src/fast_livo/include/LIVMapper.h` 与 `src/fast_livo/src/LIVMapper.cpp`，两者各自与 G2 的 B0 哈希相同。其余 10 项是原现场候选没有复制的相机目录、板端说明和启动/停止入口，具体名单在 `VERSION-IDENTITIES.json`。B0 分支沿用 A 的维护共用文件以便独立构建，并保存 B0 冻结补丁与候选 ELF。

## 工程内的本地设备参数

板端原始 A 的逐项哈希保存在 `records/maintained_source_sha256.json` 和 `STAGE-A-MAINTAINED.csv`。原板端 README 字节原样保存在 `docs/board-operation.md`。上传的工作树有以下四处设备识别改动；各分支的对应新 SHA256 在 `deploy/release.json`。

| 路径 | 维护改动 |
|---|---|
| `runtime/sensors/radar_device.py` | 从本地环境取雷达序列号，缺失时在打开套接字前退出。保留应答序列号匹配检查，落盘只写校验结果 |
| `camera/camera_sync.cpp` | 由环境或原有 `--serial` 选择相机，未指定时在初始化之前返回 |
| `camera/camera_device.hpp` | 相机参数快照用通用校验标记替换含设备信息的掩码，保留唯一匹配检查 |
| `runtime/hardware.py` | 实时采集入口在访问传感器前检查两个本地变量 |

原设备部署的 ELF 直接取自板端 install 路径。重建维护源码时，架构、编译器、路径和本地修改都可能影响输出。恢复固定 ELF 应按仓库首页校验和安装。

## 地图归档与标定

从 `/data/robot/maps/fast_livo2` 下 `runs`、`stage_a_20261123`、`stage_b0_20261123` 收录了分别 7、7、8 个带 `map/result.json` 的会话。20 个结果为 complete，另外 2 个为 incomplete，均为零位姿、零点。每份 `records/maps/*.json` 记录相对会话路径、实际存在的原始结果文档 SHA256，以及其中选定的状态、点数、位姿数、重力校验、输出文件已有哈希、资源和回放计数。地图和轨迹的摘要来自归档清单，本轮没有重新下载或重算原始观测、地图和轨迹。

B0 三次 fixed 手持回放各有 576 个位姿，地图和轨迹的归档摘要分别一致。两段尾部局部一致性变差，缺少独立真值，B0 保持研究状态。A 与 B0 的离线质量细节见 `records/stage_a/`、`records/stage_b0/`。

板端 `/data/robot/calibration` 在本轮检查时为空。实际生效的 `configs/calibration/` 与板端运行树相符，其来源用已有的 `calibration_v1` 冻结档案核对。相机内参为 11 月 10 日 v2；雷达至相机外参来自 A/B/C 三场景联合标定，联合 RMSE 5.085 mm。D 独立静态场景 4 对有效对应点的 RMSE 为 4.872 mm，投影 RMS 为 1.970 px。保留跑图参数、ROS camera_info、FAST-Calib2 导出和 ABC/D 验证摘要。绝对曝光时间对齐、跨设备时钟正式验收及整机定位精度仍需各自的独立依据。

## 工作站验证

Ubuntu 22.04 WSL，ROS 2 Humble，amd64。通过本仓的 CMake 工程独立编译安装 Livox SDK、livox_ros_driver2、vikit_common、vikit_ros 和 A 算法可执行文件。独立 CMake 构建使用 `ROS_EDITION=ROS2`、`HUMBLE_ROS=humble`、对应 CPU 的 SDK、同一份 Sophus 和 `-fno-fast-math`。本机缺少 colcon，核查了 `build.sh` 的各阶段目标、脚本语法和产物安装位置。海康相机的两个 C++ 入口使用本机 ROS/OpenCV 与板端对应的厂商头文件通过语法编译；厂商 ARM64 动态库的本机链接和真实设备采集由目标机验证。

从另一独立源码/构建目录，将通过板端摘要核验的两处 B0 源码叠加到相同算法基线，也成功编译、链接并安装了 B0 可执行文件。A 的数字回归目标 `strict_math_regression`、`geometry_regression` 均已编译并运行通过。工作站重建产物为 amd64，随仓交付的两个 ARM64 ELF 则各取自原板对应安装目录。

运行管理 6 项、重力归档 6 项和设备身份闭锁 3 项在本机通过。运行管理测试使用合成数据和桩进程。没有在本轮启动真实算法或进行实车、传感器动态复验。可在已加载 Humble 的 Bash 执行以下离线回归。

```bash
python3 tests/runtime_checks.py
python3 tests/gravity_checks.py
python3 tests/test_local_sensor_identity.py
python3 tools/verify_release.py
```
