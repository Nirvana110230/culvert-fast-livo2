# 离线 SE(3) 回环与地图重建

输入是本次前端导出的 `keyframes/index.csv`、以 IMU 为局部坐标系的逐关键帧二进制 PCD、同一回放的 `gravity_events.jsonl`。索引中的 `color_cloud` 指向对应关键帧的 RGB 点，`states.csv` 提供原始稠密 LIO 更新。后端与前端通过这些文件交换数据。单独运行后端不会改变前端里程计。

## 板端运行

板端采用 Ubuntu 22.04 AArch64、系统 Python 3.10、Pillow 与 PyYAML。数值依赖通过 `arm64-wheel-lock.json` 锁定，在独立目录核验 SHA256 后安装。GTSAM 使用有 CPython 3.10 ARM64 wheel 的 4.3a1 预发布版本。系统 Python 的搜索路径只对本进程临时扩展。

```bash
R=/data/robot/livo_pgo_dev
D="$R/backend_dev"
cd "$R/candidate"
# 首次使用时安装到空前缀。已缓存的四个 wheel 可直接离线安装。
python3 backend/private_deps.py --wheels "$D/wheels" --prefix "$D/pydeps"
# 需要从锁定的 PyPI 链接获取缺少的 wheel 时，为上一条命令添加 --download。
export PYTHONPATH="$D/pydeps" OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
python3 backend/run.py --index "$R/b1/keyframes/index.csv" \
  --gravity "$R/b1/gravity_events.jsonl" --out "$D/combined01" \
  --p0-windows 10 60 1775 1825 --mode combined
python3 backend/verify.py --index "$R/b1/keyframes/index.csv" --result "$D/combined01"
```

每次运行使用新输出目录，成功完成后才写 `summary.json`。供其他场地使用时，只有现场确认的复位窗口才传给 `--p0-windows`。时间默认相对首关键帧，用 `--time-origin-ns` 可明确改为首有效前端状态。时间轴、重力记录与索引 SHA 见 `plan.json`。支持 `--mode tape`、`--mode automatic`、`--mode combined`。在同一个索引上保持全部参数相同，可以对照三种图。自动模式只使用从历史轨迹查询的配准约束。

## 实际 ROS 2 动态 TF 旁路

建图旁路以只读方式跟随前端 `index.csv` 的完整换行记录。接收新帧后始终用该次 **LIO 更新的测量整数纳秒** 发布 `/tf` 中的 `map → odom`。初值为当次重力旋转，回环求解成功后更新校正，直到下一次有效回环持续使用当前校正。运行中只读取已经提交的帧。当前候选等待前后第 2 个关键帧到齐后验证，每 15 s 测量时间至多触发一次查询，每个查询至多两个分时历史候选。第一组具备历史回访资格时，用当时确实到达的数据冻结六轴权重样本，随后复用离线 GICP、邻组检验、方向性协方差、Huber 和图一致性求解。输出 `issued.csv`、逐查询 `graph.jsonl` 及结束摘要。停止标记点之前尚未匹配的末两帧保持已有校正。

```bash
source /opt/ros/humble/setup.bash
R=/data/robot/livo_pgo_dev
export PYTHONPATH="$R/backend_dev/pydeps${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
python3 "$R/candidate/backend/live_tf.py" \
  --index "$R/b1/keyframes/index.csv" --gravity "$R/b1/gravity_events.jsonl" \
  --complete "$R/b1/keyframes/complete.json" --rebuild --out "$R/backend_dev/live_tf_example01"
```

默认 `map` 表示本次建图的后验重力坐标系，无跨会话全局原点或北向承诺。默认 `odom` 数值对应同会话的 `camera_init`，与现有导航适配器的 `T_odom_camera_init = I` 契约一致，因此广播 `T_map_body_opt × inverse(T_camera_init_body_raw)`。适配器的 `odom → base_link` 使用同一 LIO 测量时轴；如果另一个桥改用 ROS 发布时间、墙钟或重新定义里程计原点，须在该桥显式建立时间/空间对应。TF 默认子帧是 `odom`，正式归档显示链原有 `map_gravity → camera_init` 仍由归档器拥有。若导航仲裁器正在发布同一 `map → odom`，将旁路放入独立 ROS_DOMAIN_ID 或使用单独的非冲突帧名；同一 TF 子帧只能有一个权威发布者。旁路不发布导航命令或前端状态修正。

实际独立订阅测试可在专属 ROS 域运行。它按归档原顺序逐行提交索引，使用同一组不可变小云，从另一个 ROS 进程读取 `tf2_msgs/TFMessage`，比对每个时间戳和七个位姿分量。B1 验证 1,527 条全部匹配，首末校正平移相差 2.350 m。12 ms 加速投递用于验证到达次序、算法因果性和 TF 总线回读。端到端设备运行延迟仍需在实际采集时实测。

```bash
ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1 python3 "$R/candidate/backend/test_live_tf.py" \
  --index "$R/b1/keyframes/index.csv" --gravity "$R/b1/gravity_events.jsonl" \
  --out "$R/backend_dev/wire_new01" --rebuild
```

生产者正常写出 `complete.json` 后，旁路核对其声明帧数并读完最后一条提交。启用 `--rebuild` 时，用**最终在线接受的边集合**和因果权重对整段再解一次，生成标准 `summary.json`、两套历史 TUM、前后几何和 RGB PCD。`issued.csv` 保留当时真实广播，`map_to_odom.tum` 记录收尾后回溯重建。`view.sh --pgo <旁路输出> --check` 和 `--before --check` 可从标准入口分别核查。实际开始采集前正常结束且零帧时，写 `no_initialized_keyframes` 并退出。生产者声明非零帧但缺初始化会写 `incomplete_session` 并返回错误。主动中断时保留已发布的日志，不将中断的会话标为完整历史重建。

## 官方 STD 候选试验

`std_probe.py build` 从固定的 hku-mars/STD 提交 `6e5903cfa2aaed5ca3c9e7d21424eebcca7655bb` 获取并核验头文件、实现、Livox 配置及 LICENCE，另存原件并生成独立描述子/检索适配源。保留 GenerateSTDescs、SearchLoop、AddSTDescs 及三角形/平面验证，移除 ROS1 参数和可视化、未调用的后置 Ceres 优化，对不足 K 的小云限制近邻遍历上界。板端 GCC、Eigen、PCL 1.12 与 FLANN 编译为独立进程。固定源码 LICENCE 为 GPLv2，README 另声明个人和学术免费、商用需联系作者。[固定源码](https://github.com/hku-mars/STD/tree/6e5903cfa2aaed5ca3c9e7d21424eebcca7655bb) · [许可](https://github.com/hku-mars/STD/blob/6e5903cfa2aaed5ca3c9e7d21424eebcca7655bb/LICENCE)

```bash
python3 backend/std_probe.py build --out "$D/std_build_new"
python3 backend/std_probe.py prepare --index "$R/b1/keyframes/index.csv" \
  --gravity "$R/b1/gravity_events.jsonl" --plan "$D/b1_joint/plan.json" --out "$D/std_inputs_new"
OMP_NUM_THREADS=1 "$D/std_build_new/std_probe" "$D/std_inputs_new/list.txt" "$D/std_new.csv"
python3 backend/std_probe.py audit --result "$D/std_new.csv" --groups "$D/std_inputs_new" --out "$D/std_audit_new.json"
```

每十个关键帧组成一个子图，先按实际姿态消除倾装影响，再转为重力轴对齐、中心在当前关键帧的局部坐标。配准候选子图仍屏蔽冻结留出。STD 输出仅作独立提议试验，未写入回环位姿图。

## 候选、验真和图优化

- 使用同一回放的 `runtime/gravity.py` 计算固定重力旋转，将 `camera_init` 姿态换到 `map_gravity`。如果有前端元数据，同时核验父子帧、配置、可执行文件与重力事件的会话身份。
- 自动候选默认来自半径 5 m 内、时间差至少 75 s 且累积路程差至少 18 m 的历史关键帧。当前轨迹每行进 1.8 m 或 15 s 检索一次，每个查询至多保留两处有时间间隔的历史观测。跨整段时间均匀取最多 45 个查询，`plan.json` 保留选择。P0 从两段现场确认窗口各取接近位置中位数的关键帧，分别记录静止半径。其相对变换由点云求解。
- 原始里程计给 GICP 提供初值。P0 还试验零平移以及单位初值，最终使用 GICP 变换。主配准取中心与前后第 2 帧组成子图，另一组仅用前后第 1 帧交叉检验。配准体素 0.18 m、对应上限 0.9 m，两向覆盖单独统计。默认门限为两向 0.32 m 内覆盖率均至少 0.38、内点 RMS 至多 0.19 m、法向二阶矩最小特征值至少 0.035、归一化点面六自由度最小特征值至少 0.0015。邻组结果须收敛并与主结果相差至多 0.30 m、5°。额外检查相对初值的修正、竞争历史位置以及重复使用当前观测。每次失败原因写入记录。
- GTSAM Pose3 使用相邻里程计与通过的回环 BetweenFactor、首姿态先验、Levenberg–Marquardt 和分块 Huber 损失。六轴权重取自本次回放，下节给出计算过程。对首轮或后续图解中白化残差范数仍大于 7.2 的边，保留拒绝记录后重新求解。不存在可接受约束时保留原轨迹，结果仍可用于诊断。
- 所有门限基于这一室内试验设置。无真值身份的自动接受边只能作为几何假设。重复走廊可能导致多个近似等价位置。候选范围之外的漏检同样不会进入精度分母。

## 六轴权重依据

在本次轨迹均匀选取至多 48 对相邻关键帧，只使用剔除留出区域后的点。取 GICP 已收敛且双向 0.32 m 内覆盖均超过 45% 的样本，将实际里程计相对位姿与该帧间 GICP 差异投到源帧 Pose3 右切空间，顺序为 `rx,ry,rz,tx,ty,tz`。按关键帧触发规则，以距离/1 m、转角/15°、间隔/5 s 三者最大值的平方根归一化，最短更新取 0.2 的方差下限。每轴取归一化残差的 1.4826 倍 MAD 与绝对残差第 68 百分位的最大值，再与配准离散误差参考值按平方和开方合成。平移参考值取 0.18 m 体素除以有效 1 m 空间格数的平方根；格数取实测中位数并保守封顶 64。角度按实测对应点 RMS 臂长的中位数换算。少于 8 个有效对时使用两倍工程下限并标记 fallback。实际相邻因子再按各自运动量缩放。

每个回环从中心组和邻帧组求出两个 SE(3) 结果，用两解之差的绝对切向分量除以 √2 作为轴向重复性尺度。再把点距 RMS 除以有效空间八分区数的平方根，按观测点的实际 RMS 臂长换算角度尺度。每轴保守取上述参考量与相邻帧估计的最大者。将归一化点面雅可比转到源帧，强方向维持该尺度，弱方向的方差按照中位特征值/该方向特征值放大 1–16 倍，用含非对角项的完整 6×6 协方差装配回环。

`noise_model.json` 记录每次相邻帧配准、逐轴 MAD、P68 及最终基础尺度。`edges.json` 记录邻组差异、观测矩阵、每轴尺度和实际完整协方差。同袋 GICP 与前端共享测量，所得量描述组内一致性。工程下限、放大上限及运动量缩放仍是建模假设。冻结全部实际入图边和相邻因子，仅把回环标准差统一乘以 0.5、1 和 2，重新求解并报告轨迹、P0 与独立回访点距对权重的敏感程度，见 `weight_sensitivity.json`。

## 独立回访几何

读取点云、拟合回环前，根据原始轨迹选出至多五对回访。在两次时间索引周围各保留前后两关键帧，并从所有配准子图剔除两次估计位置各 4 m 半径的竖直柱体。不同留出位置至少间隔 12 m，P0 周边保留给端点试验。未参与回环拟合的点与区域仍来自同一袋，里程计相邻因子使用此前端输出。

每个留出区域在旧访次单关键帧建立固定参考，在新访次三个关键帧分别按优化前后位姿重投影。输出双向点距和覆盖、地面及两个主方向墙面的有符号偏移、两访次等权厚度 P90−P10、墙距及墙角。墙面孔洞候选要求三个高度层都缺少回波且两侧有连续支撑，只有独立现场标注可认定门口身份。表面覆盖过低时，指标不适合作为同一物理面的误差。未识别到同一结构时留空。

`holdout_validated.json` 是几何数值引用入口。每面、每阶段的 `quality.surfaces.<名称>.<阶段>.published` 仅在原始和法向一致子集的拟合均通过时填值，失败时为 null，并列明 `reasons`、原始拟合及复核拟合。统一要求参考至少 42 点、新访次至少 28 点，两次拟合法向差均不超过 15°，参考足迹覆盖至少 35%，主要层占候选点至少 30%，两维切向展布至少 0.50/0.20 m。两墙间距、墙角及开口继承各组成墙的状态，任一组成面不足则不发布。阈值用于报告资格判定，未进入回环筛选与图求解。旧归档可用 `surface_quality.py --index ... --gravity ... --result ... --out ...` 只复核历史面，不改已有轨迹、地图及原始拟合文件。

## 产物与复核

| 文件 | 内容 |
| --- | --- |
| `trajectory_before.tum`、`trajectory_after.tum`、`poses.csv` | 完整关键帧、原时间戳、重力系位置与姿态 |
| `trajectory_dense_before.tum`、`trajectory_dense_after.tum` | 有 `states.csv` 时输出全部 LIO 更新。优化姿态由相邻关键帧的 `T_map_odom` 做平移线性插值与旋转 SLERP 后作用于原始状态 |
| `map_to_odom.tum`、`map_to_odom.json` | 每帧 `T_map_gravity_camera_init = T_map_gravity_body_opt * inverse(T_camera_init_body_raw)`，包含重力固定旋转 |
| `edges.json`、`factors.json`、`registration_progress.jsonl` | 候选、矩阵方向、质量、拟合前后残差、Huber 权重、拒绝原因，以及实际入图的每条因子 |
| `map_before.pcd`、`map_after.pcd` | 全部选中关键帧的局部 XYZ/强度依历史优化前、后位姿分别生成，在地图写入阶段保留全部有限点 |
| `map_rgb_before.pcd`、`map_rgb_after.pcd` | 各关键帧同时间 RGB 小云单独变换并完整写出，颜色位逐点保留 |
| `holdout.json`、`summary.json`、`map_comparison.png` | 独立空间留出指标、资源/磁盘/轨迹统计及同尺度 XY/XZ 投影；投影范围由轨迹包围盒外扩 12 m，PCD 保留范围外几何 |
| `verification.json` | 独立读回全部 PCD、逐关键帧复算所有点、核对 RGB 位和强度、检验 `T_map_odom*T_odom_body`，记录各图哈希 |

## 上游对照

liangheming/FASTLIO2_ROS2 的 `pgo` 子包使用半径与时间候选、PCL ICP、GTSAM ISAM2 和 Pose3 BetweenFactor。核查提交 `7baa59f7339ba88ce3b646fe35b23f61031c5d3f`，包内 `LICENSE` 为 MIT 式授权正文，SHA256 `e4da13b8ee8adf3c184f3cfaa765c1b002efd9e1b7a200fe7c507a30391f8fb9`。`simple_pgo.cpp` SHA256 为 `f3df13f99d97e079c29f508ee620a900f06cac7d47b22aeb9ec59c98ee221e1b`。[原始实现](https://github.com/liangheming/FASTLIO2_ROS2/blob/7baa59f7339ba88ce3b646fe35b23f61031c5d3f/pgo/src/pgos/simple_pgo.cpp) · [包内许可](https://github.com/liangheming/FASTLIO2_ROS2/blob/7baa59f7339ba88ce3b646fe35b23f61031c5d3f/pgo/LICENSE)

Hero941215/fast_lio-sam_loop 的 ROS1 主包在 `laserMapping.cpp` 中做时间/距离检索、历史关键帧子图 ICP、ISAM2 和轨迹/地图校正。核查提交 `f9139cb1be19d4893899de64950c323f8cb9cff5`；包内 GPLv2 正文 SHA256 `8177f97513213526df2cf6184d8ff986c675afb514d4e68a404010521b880643`。源文件 SHA256 `dc994cd0506bafa27d87e86205c9396d2a3eb6bb7156580074e1e3aed8c3a2c3`。本目录独立实现数据接口、验真、图装配与重建，通过公开 Python API 使用 small_gicp 和 GTSAM。[原始实现](https://github.com/Hero941215/fast_lio-sam_loop/blob/f9139cb1be19d4893899de64950c323f8cb9cff5/fast_lio-sam_loop/src/laserMapping.cpp) · [包内许可](https://github.com/Hero941215/fast_lio-sam_loop/blob/f9139cb1be19d4893899de64950c323f8cb9cff5/fast_lio-sam_loop/LICENSE)
