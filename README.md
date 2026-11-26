# 香橙派 FAST-LIVO2 维护候选

本地交接包含已核对的阶段 A 运行管理代码与回归夹具。完整 245 项维护文件的 SHA256 与 Git blob 标识列在 STAGE-A-MAINTAINED.csv。SOURCE-FILES.csv 可核对本仓原样收录的项目文件。正式源码的原始提交、文档提交以及 B0 的隔离补丁与历史 ELF 记录分别列在 VERSION-IDENTITIES.json。

完整 Git 历史与直接指向树的隐藏引用在受限介质 orange/all-refs.bundle。恢复源码时从版本清单读取 source_commit，检出此提交下的 fast_livo2 子树。文档提交另有发布记录和增量补丁。orange/orangepi.zip 保存本机当前工作树及运行证据。B0 原补丁位于该归档 docs/mainline/2026-11-23/stage_b0/evidence/startup_imu.patch，保持隔离。

本地候选仅为维护子集。完整构建与设备恢复须使用受限源码及配套依赖。算法、传感器通信配置、冻结标定、实验日志、原始数据与厂商代码均通过受限档案交接。fast_livo、vikit、Livox 的许可应分别核定。当前禁止上传。
