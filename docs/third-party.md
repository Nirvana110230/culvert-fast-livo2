# 第三方来源及许可

| 组件 | 来源与版本 | 仓内许可依据 |
|---|---|---|
| FAST-LIVO2 | HKU-MARS 算法，工程继承 Robotic-Developer-Road 的 Humble 移植 `837b7bbc1431cb04cf936528e52c83c835efba8e`；再经工程自己的 A 与 B0 修改 | `src/fast_livo/LICENSE` 是 GPL v2 文本；同一源码树的 `package.xml` 声明 BSD，两处存在冲突 |
| Vikit | WilsonGuo/Fast-LIVO2-Drvier-ROS2 的 `rpg_vikit`，提交 `2b98dd8e3b3054333bbbefb790ddad51eb14fa65`，工程适配 ARM 与 ament | 两包 `package.xml` 均声明 GPLv3，源文件保留作者头 |
| Sophus | strasdat/Sophus 1.22.10，`de0f8d3d92bf776271e16de56d1803940ebccab9` | 板端固定安装内容及同一板端来源树的 `deps/sophus/LICENSE.txt` |
| Livox ROS Driver2 | 工程继承经验证的 Mid-360S 专用快照，原包字段 1.0.0，CMake 版本 1.2.5。本地改动含时间共享。历史输入归档 SHA256 为 `aa27b856ccf63b0ee0db2272b2504b3a2ab206e619bfb5939fa5282e9373deac`；没有可证明一致的上游提交 | `src/livox_ros_driver2/LICENSE.txt` 分项列出原许可，RapidJSON 的原文另附在 `3rdparty/rapidjson/license.txt` |
| Livox SDK2 | 同一 Mid-360S 快照的板端源树 `/home/orangepi/lio_base/2026-10-28/src/livox_sdk`，运行 API 1.3.0，系统 spdlog/fmt 链接改动已包含在源树中 | `vendor-src/livox_sdk/LICENSE.txt` 和其 3rdparty 子树的原文，适用于相应部分 |
| 海康 MVS | 海康机器人 MVS 5.1.0 Linux aarch64，由使用者通过厂商渠道取得 | 厂商安装包的许可随介质交付，本仓仅提供获取方法、版本及逐文件摘要 |

本次在用户授权的私有工程中保留各组件的原始版权、许可和包描述。项目没有重新声明第三方代码的许可证。后续公开发布或再分发时，FAST-LIVO2 的许可冲突需要向其权利人核实。

[算法原项目](https://github.com/hku-mars/FAST-LIVO2) · [ROS 2 移植](https://github.com/Robotic-Developer-Road/FAST-LIVO2/tree/837b7bbc1431cb04cf936528e52c83c835efba8e) · [Vikit 来源](https://github.com/WilsonGuo/Fast-LIVO2-Drvier-ROS2/tree/2b98dd8e3b3054333bbbefb790ddad51eb14fa65/rpg_vikit) · [Sophus](https://github.com/strasdat/Sophus/tree/de0f8d3d92bf776271e16de56d1803940ebccab9) · [Livox SDK2 项目](https://github.com/Livox-SDK/Livox-SDK2) · [Mid-360S 公开参考](https://github.com/WilsonGuo/Fast-LIVO2-Drvier-Mid360S)
