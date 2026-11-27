# 估计器关键帧接口

设置 `FAST_LIVO_KEYFRAMES` 为本次运行的独立空目录。正常结束后，`complete.json` 给出已写入的关键帧数和状态记录数。`metadata.txt` 记录选择参数、坐标系、运行配置与可执行文件 SHA256、实际加载的外参及相机模型参数。直接 Bag 回放通过 `runtime/offline.py` 设置这些身份信息。

## 空间与时间

`index.csv` 每行对应一次已完成 IMU 初始化、具有有效下采样点及平面匹配的 LIO 更新。`stamp_ns` 从估计器已同步的 `lio_time` 双精度秒数四舍五入得到，在当前数据历元下分辨率受双精度限制。姿态为该参考时刻的 `T_odom_imu`，`odom` 使用估计器的 `camera_init`，四元数顺序为 xyzw。单独的 `gravity_events.jsonl` 将本次初始化绑定到惯性重力方向。

`cloud` 指向相对路径的二进制 PCD。去畸变后的每个雷达点使用 `R_imu_lidar p_lidar + t_imu_lidar`，保存为参考时刻 IMU 局部坐标下的 xyz 与强度。这个接口保留所有有限坐标点。首帧之后，相对于上个关键帧，平移达 1.0 m、旋转达 15°、或经过 5.0 s 都会触发采样。分别可用 `FAST_LIVO_KF_DISTANCE_M`、`FAST_LIVO_KF_ANGLE_DEG`、`FAST_LIVO_KF_INTERVAL_S` 设置正阈值。

启用相机时，导出器保留本帧点云直到同参考时刻的 VIO 图像就绪。实际 Vikit 模型将 `R_camera_imu p_imu + t_camera_imu` 投影到当前处理分辨率下的 BGR 图像。每个像素保留其内深度最小的有效回波，`color_cloud` 使用相同 IMU 坐标与 PCL 的 packed RGB。相机未启用、对应图像缺失或无可见点时，颜色路径为空，索引保留具体原因。不同时间的图像不会补配。

索引另含去畸变输入点数、保存点数、下采样点数、匹配平面数、匹配点到平面平均绝对残差、着色点数与图像测量时刻。后台写盘线程先写点云临时文件并更名，再提交索引行。写盘异常或队列耗尽会报错。正常收尾排空队列，再生成完成标记。

## 完整估计状态

导航适配器从 `/culvert_source/state` 读取 `camera_init → imu` 的后验位姿，从 `/culvert_source/scan` 读取 `livo_lidar` 坐标的去畸变 XYZ/强度点云。两者取同一个估计时刻；有订阅且该更新通过传播终点和点时间核对时才发布。关键帧 PCD 则已施加 LiDAR→IMU 外参，可直接与索引中的 IMU 位姿相乘。地图重建使用索引的 LIO 阶段，与后续 VIO 阶段分别存储。

`states.csv` 按实际处理阶段输出位姿、速度、重力、陀螺/加速度偏置及姿态/位置方差对角线。阶段 `LIO` 和 `VIO` 表示有效的对应更新，`LIO_no_plane_match`、`VIO_no_update` 区分未形成约束的处理。原配置启用 IMU 传播发布时，附加 `IMU_propagated`。LIO 和 VIO 可共享同一测量时刻，读取轨迹应明确选择阶段。

## 数值回归

在已经构建的 FAST-LIVO2 工作区执行下列命令。测试使用项目的 pinhole 相机实现、非单位外参、前后景遮挡与错戳图像，直接检查导出的二进制坐标、RGB 与选择原因。

```bash
cmake --build build/fast_livo --target keyframe_export_regression
build/fast_livo/keyframe_export_regression
```
