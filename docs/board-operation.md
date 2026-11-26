# FAST-LIVO2 香橙派主线

板端工作目录为 `/home/orangepi/fast_livo2`。日常从这个目录执行以下命令。脚本自行加载 ROS 2 Humble、主线安装目录和所需库。现场开始前接好供电、雷达、相机和同步器，将支架静止放好，让 IMU 完成初始化。

| 操作 | 命令 |
|---|---|
| 持续建图 | `./start.sh` |
| 查看会话、资源和视觉更新 | `./status.sh` 或 `./status.sh --json` |
| 实时查看 | `./view.sh`（RViz），或 `./view.sh --foxglove`（回环端口 8765，使用支持 `foxglove.sdk.v1` 的客户端） |
| 正常停止并等待归档 | 在同板另一个终端执行 `./stop.sh` |
| 通过同一算法与归档链回放 | `./replay.sh <bag目录>`，需要手动停止时加 `--wait-stop` |
| 打开或检查保存的会话 | `./view.sh --run <会话目录>`，仅校验加 `--check` |
| 打开已导出的水平地图 | `./view.sh --map <含manifest.json的地图目录>` |

从 Windows 通过 USB 查看 Foxglove 时，在 Windows 终端执行 `ssh -N -L 8765:127.0.0.1:8765 opiss-usb`，然后在客户端连接 `ws://127.0.0.1:8765`。

会话位于 NVMe 的 `/data/robot/maps/fast_livo2/runs/`。会话名使用 UTC；参数、程序 SHA256 和 IMU 初始化来源都在本次会话内。正式产物为 `map/map_rgb.pcd`、`map/map_geometry.pcd` 和 `map/trajectory.tum`。三者均已实际转换到 `map_gravity`，第三方直接读取文件即可。正式归档提交以 `map/manifest.json` 与 `map/result.json` 的成功状态为准；出现错误时按结果中记录的位置取回留存数据。

算法在 `camera_init` 估计，归档进程用本次 IMU 初始化的重力建立 `map_gravity → camera_init`。实时查看请选择固定坐标 `map_gravity`。水平来自重力方向，保留地形坡度和原点，航向没有绝对参考。实时显示有容量限制，正式写盘保留收到的原生点记录、颜色等字段及完整轨迹。查看器可以随时关闭。

算法固定由 `install/fast_livo/lib/fast_livo/fastlivo_mapping` 执行，只消费本次 `resolved.yaml`。叠加顺序见 `runtime/run_files.py` 和会话的 `parameters_manifest.json`。相机内参及 ABC 联合外参在 `configs/calibration/`，历史来源在 `records/calibration_sources.json`。维持原 Line0 触发、timeshare 图像、0.1 秒补偿；修改安装或光学配置需重新核验对应标定。

修改代码后执行 `./build.sh`。该脚本从 `src/`、`camera/` 构建，依赖系统 Humble、OpenCV/PCL 和本树 `deps/` 的已固定 ARM64 第三方文件。海康自带的 libusb 仅用于相机进程。`./build.sh tests` 运行真实视觉更新与内存边界回归。局部视觉地图按距离、年龄和系统可用内存回收；累计输出写入会话数据盘。没有固定建图时长。实际运行记录在本目录 `records/`。
### A 阶段参数与离线回归

LIO 迭代上限使用 `lio.max_iterations`，曝光估计使用 `vio.exposure_estimate_en`。当前正式配置分别为 5 和 false。旧名 `lio.min_iterations` 和 `imu.exposure_estimate_en` 仍可单独使用，启动时会提示迁移。新旧名同时给出且值不同时，程序报告两个键和值并退出。省略整个曝光开关时沿用历史代码缺省 true。

`imu.b_gyr_cov` 和 `imu.b_acc_cov` 各自向三轴偏置过程噪声对角块提供同一个标量，参与每步的 `dt²` 缩放。正式配置两项均为 0.0001。相机补偿仍由 `time_offset.img_time_offset` 单独控制。

可用 `FAST_LIVO2_REPLAY_DOMAIN=207 ./replay.sh <bag目录>` 将回放放到独立 ROS 域。回放仍使用会话自己的配置快照、参数回读、算法进程身份和重力归档。构建脚本将编译诊断中的源码路径映射到规范维护目录，便于隔离构建与正式构建逐节比较。
