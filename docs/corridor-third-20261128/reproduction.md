# 第三轮走廊实验复算说明

归档日期为 2026-11-28，时区 Asia/Shanghai。正式报告使用的原数据身份为 `corridor03_20261128T072245Z_tzunG9`。结果文件、轻量脚本可以随本资料单独阅读；原始扫描、图像、完整地图和全量状态取自保留的实验档案。以下 PowerShell 示例从本分析资料目录执行。本机采用 Python 3.13、NumPy、SciPy、pandas、Matplotlib、Pillow、PyYAML。结构支撑提取沿用已部署候选中的 ARM64 Python 依赖。

```powershell
$work = 'E:/涵洞巡检轮式机器人/orangepi-dev'
$repo = Join-Path $work 'livo-pgo-dev'
$maps = Join-Path $work 'data/corridor_runs/20261128_third_mapping'
$analysis = (Get-Location).Path
$env:PYTHONUTF8 = '1'
```

## 只用资料包复核

`floor_distribution.csv` 给出三套分母下逐阶段的有符号残差极差和地面高度分位。基于 `geometry/floor_cells.csv`，按 cohort、stage 分组，对 residual_z_m 平方取均值开根号，可以复得报告核心结论。五区独立分析表在 `loops/holdout_bidirectional.csv`。每行固定方法、方向和区域，不宜混合正反向的分母。1,610 个训练块和 409 个留出块的三点误差保存在 `structure_eval/blocks.csv`，每阶段各 2,019 行，按 split、stage 分组，把三个误差列展平计算 RMS。已合并各专题 JSON 的 `metrics.json` 约半兆量级。可在独立结果目录从各专题 JSON 和地面表重建总指标。

```powershell
python collect_metrics.py . --out "$env:TEMP/corridor03-metrics"
```

## 重算全图及原生点追踪

A/B/C 的定义以报告的版本表为准。`spatial_5cm` 已有本轮真实产物。首次从全量图准备新副本时，脚本 `maps/export_5cm.ps1` 的参数为 `-MapRoot`、`-CloudCompare`，后者给出 2.13.2 可执行文件。下面的计算直接使用现有 5 cm 图。输出可指向任一分析目录。相对姿态及活动时长的脚本要求输出目录已存在。

```powershell
$new = New-Item -ItemType Directory -Path (Join-Path $env:TEMP ("corridor03-" + [guid]::NewGuid()))
python maps/compare_maps.py $maps $repo $new.FullName
python maps/check_rigid_rows.py maps/frame_readback.csv $maps $repo $new.FullName
python maps/static_orientation.py $maps $new.FullName
python maps/movement.py "$maps/pgo" $new.FullName
python geometry/evaluate.py --data $maps --out "$new/geometry"
```

`check_rigid_rows.py` 使用 CSV 的 id、stamp_ns、points、color_points 四列，也可直接传原生 keyframes/index.csv。逐帧回读只检查点云数据关联与位姿变换。`geometry/evaluate.py` 调用算法仓中的 `tools/evaluate_z_geometry.py`。在原工作区或包含本资料的独立算法仓使用该脚本；独立展开 ZIP 时可将资料目录置于算法仓内以便模块发现。

## 原包时序、前端及在线回环

对原始 Bag 的读操作只在板端进行。分片按数字编号、消息按 SQLite id 定序；IMU 读必要矢量，Image、LiDAR、CameraInfo 仅读 Header 的固定前缀，相机小 JSON 消息读取相关字段。详细取数命令、上一轮对照依赖及 sysstat 查证见[前端复算](frontend/reproduce.md)。对已有逐次静止更新文件运行 `frontend/extra_static/check_state_steps.py` 可复核 49 个时刻的偏置阶跃。视觉补充程序的四个输入见[后验诊断复算脚本](frontend/visual_uncertainty.py)。

以下命令假定 `loops/raw` 已有授权取得的在线归档和 20 个重访关键帧云。缺少时，`python loops/fetch_inputs.py --data $maps --ssh opiss-usb` 只提取脚本所列文件。回环分析读取现成边与轨迹，在相同源、目标回波上分方向重新打分，全程不需要配准或图优化。

```powershell
python loops/analyze.py --data $maps
```

## 共享面及惯性方向的复算

`structure_eval/observations.json` 是从原生支撑、方向和三条已求解轨迹得到的逐观测残差。Windows 汇总可单独执行，`$maps` 需要有 `structure/shared_plane_associations.json`、`structure/summary.json` 和 `pgo/trajectory_after.tum`。图中有符号变化统一取 C 减 B。

```powershell
python structure_eval/summarize.py structure_eval/observations.json $maps $new.FullName
```

从原生关键帧重新提取这份中间结果，可在已有候选的 Linux 板上使用以下入口。脚本以标准输出写 JSON，可通过 SSH 传回 Windows；只重新计算残差，不调用任何图求解。对同一个 FIELD 的 `plan.json` 使用其中记录的关键帧及重力文件。下列 Windows 命令将脚本从标准输入传给远端，将结果保存在新建的临时目录。

```powershell
$target = '/data/robot/livo_pgo_dev/candidate_z_20261128'
$archive = '/data/robot/maps/fast_livo2/runs/corridor03_20261128T072245Z_tzunG9/full_z_5R7iGB'
$envcmd = "PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 PYTHONPATH=$target/backend:$target/backend/pydeps python3 - $archive"
python -c "import pathlib,subprocess,sys; f=pathlib.Path(sys.argv[2]); o=pathlib.Path(sys.argv[3]); r=subprocess.run(['ssh','-o','StrictHostKeyChecking=yes','-o','BatchMode=yes','opiss-usb',sys.argv[1]],input=f.read_bytes(),stdout=subprocess.PIPE,check=True);o.write_bytes(r.stdout)" $envcmd structure_eval/audit_surfaces.py "$new/observations.json"
```

## 图集与地图位置

41 张原始 PNG 在 `D:/桌面/建图效果/第三轮走廊建图`，工程分析目录的 `visual/originals` 另存了完整副本。本地 `visual/sheets` 的 8 张索引图覆盖全部原图。可用 `python visual/review_images.py <原图根> <输出目录>` 从 PNG 重建索引与报告缩图。全点投影、地面统计及局部薄层均使用未另行配准的三阶段文件。

| 文件 | Windows 实际路径 |
|---|---|
| A，前端 5 cm | `E:\涵洞巡检轮式机器人\orangepi-dev\data\corridor_runs\20261128_third_mapping\spatial_5cm\A_geometry.pcd` |
| B，PGO 5 cm | `E:\涵洞巡检轮式机器人\orangepi-dev\data\corridor_runs\20261128_third_mapping\spatial_5cm\B_geometry.pcd` |
| C，联合 5 cm | `E:\涵洞巡检轮式机器人\orangepi-dev\data\corridor_runs\20261128_third_mapping\spatial_5cm\C_geometry.pcd` |

同目录有三个 `*_rgb.pcd`。原生全量地图位于相邻的 `pgo`、`structure` 两目录。
