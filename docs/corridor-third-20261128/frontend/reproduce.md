# A/B 统计复现

Python 3.11 或更新版本用于本地校验和分析，本次在 3.13.5 运行。分析使用 NumPy、pandas、SciPy、matplotlib 和 PyYAML；图中文字采用本机已安装的中文字体。设备端的采集器只依赖 Python 标准库和系统 SQLite，原机 Python 3.10.12。所有命令均以已停止写入的实验归档为输入。

## 提取

从本目录执行以下 PowerShell 命令，变量取实际目录。SSH 别名须已确认对应的板卡、登录身份、架构和可信主机指纹。命令使用严格主机校验，以只读方式打开数据库。公开复现材料不附现场原始数据。

    python collect_inputs.py --alias $DeviceAlias --field $FieldDirectory --run $SessionName --raw $PrivateRawDirectory --manifest source_fingerprints.json

脚本从会话取明列的文本和 CSV 文件，生成会话字段摘录，并逐个与板端 SHA256 比对。Bag 只流式导出相机字段、IMU 三轴原始值和各话题必要的时间字段。图像、点云 BLOB 仅读前 12 字节，取得结果是七种行格式的压缩 CSV。行格式在 bag_extract.py 的 SCHEMA 中，每行以前缀“类型、分片、分片内行号、Bag 纳秒”开始。原始输入目录按如下内容组织。

    raw/
      clocks.csv.gz, metadata.yaml, recorder.log, events.txt, session_subset.json
      active_parameters.json, resolved.yaml, algorithm_identity.json, algorithm.log*
      resources.json, resources.jsonl, gravity_transform.json
      diagnostics/, Log/, keyframes/, sensors/

单独重新校验原始小文件时，在上述命令后附加 --verify-only。分片大小按实际文件的 stat 字节数求和；已核对逐片消息数，未计算原始 Bag 逐片内容 SHA256。

## 分析

    python analyze.py --raw $PrivateRawDirectory --tools $AlgorithmTools --previous $PreviousEvidence --imu $ThirdImuJson --output $OutputDirectory

AlgorithmTools 为算法版本内的 tools 目录，需含 inspect_bag_clocks.py 和 summarize_frontend_timing.py。PreviousEvidence 为第二轮候选前端的 z_geometry/front_static，使用其中的 offline.json、static_updates.json、frontend_comparison.json、imu.json。ThirdImuJson 为第三轮已归档的联合分析输出 imu.json；其两段原始 IMU 均值和消息数会与当前字段提取重新核验。输出包含 metrics.json、统计 CSV 与五张 PNG。除谱范数与分位数的显示舍入以外，以 JSON 内数值为准。

首尾比力残差的方法在本版算法仓 tools/imu_velocity_balance.py，给定原始 Bag 和会话后按首个 LIO 状态起算的秒数传 --static 5 25 2059 2079 即可生成 ThirdImuJson。此脚本依赖该板 ROS 2 的 rclpy 与 sensor_msgs，仅做归档分析。

## 统计口径

- Bag 间隔在原始分片/行序中统计。topic_clocks.csv 的 record 指 Bag 时间；source 对图像、CameraInfo、雷达、IMU 指消息头，对三种 String 话题则使用 field 列标明的主机单调时间。第一/末纳秒值不得跨时钟直接相减求延迟。
- interval_outliers.csv.gz 给出 10 Hz 流间隔小于 50 或大于 150 ms、IMU 小于 2.5 或大于 7.5 ms 的定位。相机事件按类别的设备时钟分布在 metrics.json 内。
- estimator_60s.csv 和 processing_60s.csv 均相对首个 LIO 状态；resources_60s.csv、resources_samples.csv 以会话启动为零。视觉缓存用同时留下的单调时钟关联最近的前端处理时刻。
- static_updates.csv 区分尾段日志确认与全程重算。extra_static/ 下提供前三十一窗的独立偏置阶跃对照。下一次 LIO/VIO 净变化包含中间传播和其他观测。本次日志没有保存单独陀螺更新注入后的完整协方差矩阵。

## 历史 CPU 样本

在前述已校验板端，另执行以下命令取 2026-11-28 采集窗口内按核的既有历史记录，并核对温度查询结果。

    python collect_system_history.py --alias $DeviceAlias --archive /var/log/sysstat/sa28 --out $OutputDirectory

attitude_analysis.py 使用实录 gravity_transform.json 的 R_GC，输出 2,080 个互不重叠的一秒窗；首末不满一秒的余段仅计入全速率轨迹统计。
