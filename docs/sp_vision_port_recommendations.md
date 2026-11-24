# 基于 sp_vision_25 原版的改造建议

## 1. 背景与范围

- 基线：`~/sp_vision_25-main_raw`（下称「原版」）。
- 参考实现：本仓库 `auto_aim/`（下称「本项目」）。
- 依据：2026-10-27 对两者的代码 diff。本项目自瞄 CTest 尚未在本机运行（缺 ONNX Runtime 1.28.0），以下建议是代码层面的判断，没有经过实测。
- 目标：从原版重新改出一个版本时，决定哪些改动应该移植、怎么移植、按什么顺序移植。

建议按以下四级处理：

| 级别 | 含义 |
| --- | --- |
| P0 必做 | 修复原版 bug，几乎没有代价 |
| P1 推荐 | 健壮性和工程改进，代价很低 |
| P2 按需 | 有取舍，取决于部署形态、平台和赛制 |
| 不照搬 | 本项目里的退化或不宜直接复制的做法 |

## 2. P0 必做：原版 bug 修复

| # | 问题 | 位置 | 修改 | 验证 |
| --- | --- | --- | --- | --- |
| 1 | 小陀螺判定误用 EKF 索引 8（半径） | `tasks/auto_aim/aimer.cpp` 选板函数 | `std::abs(target.ekf_x()[8]) <= 2` 改为 `std::abs(ekf_x[7]) <= min_spin_speed_`；`min_spin_speed` 从 YAML 读取，要求非负有限 | 移植 `tests/aimer_spin_selection_test.cpp` |
| 2 | 高低速延迟用带符号比较，反转目标永远走低速分支 | `aimer.cpp` | `target.ekf_x()[7] > decision_speed_` 改为 `std::abs(target.ekf_x()[7]) > decision_speed_` | 同上，补充正、反转两组用例 |
| 3 | `Target(x, vyaw, radius, h)` 构造函数成员未初始化 | `tasks/auto_aim/target.cpp` | 在初始化列表中给 `name`、`armor_type`、`priority`、`jumped`、`last_id`、`switch_count_`、`update_count_` 等赋初值；初始化顺序与声明顺序一致 | 开启 `-Wall -Wextra` 无 `-Wreorder` 警告 |
| 4 | 匹配装甲时 `int id;` 未初始化，且固定取 3 块 | `target.cpp` | `int id = 0;`；循环上界改为 `std::min(3, armor_num_)` | 移植 `tests/target_layout_test.cpp` |
| 5 | 图像中心写死 1440x1080 | `tasks/auto_aim/tracker.cpp` 两处 | 改为 `center_norm` 与 `(0.5F, 0.5F)` 比较 | 换分辨率后目标排序不变 |
| 6 | ROI 宽高为 -1 时改写成员 `roi_` 并缺少越界检查 | `tasks/auto_aim/yolos/yolov5.cpp` | 用局部变量计算有效 ROI，越界时报错并返回空结果；构造时校验 ROI 参数 | 用越界 ROI 配置启动，确认报错而非崩溃 |

注意：原版 `tools/exiter.cpp` 同时处理 `SIGINT` 和 `SIGTERM`，**新版本必须保留**。本项目删掉了 `SIGTERM`，属于退化，见第 5 节。

## 3. P1 推荐：低代价改进

### 3.1 检测健壮性与实时性

- **后处理过滤非有限值**：`yolov5.cpp` 中跳过分数、类别得分、关键点含 NaN/Inf 的候选。
- **去掉实时路径上的写盘**：删除 `detector.cpp` 的 `save()` 调用和 `yolov5.cpp` 的 `imgs/` 保存。如果仍需采集样本用于迭代分类器，改成配置开关，并且限频、放到独立线程里写。
- **`imshow` 只在 debug 下调用**：`detector.cpp` 中的 `binary_img`。
- **复用 letterbox 缓冲**：把每帧新建的 640x640 `cv::Mat` 改为成员变量，每帧 `setTo(0)`。

### 3.2 类别映射集中化

- 新增 `set_yolov5_classification()`，替换 `armor.cpp` 中两处重复的颜色、兵种、类型映射，未知类别显式归为 `not_armor`。
- 颜色顺序通过 `yolov5_color_order` 配置。**默认值应与所用模型一致**：继续使用原版 `yolov5.xml` 时默认是 `blue_red_gray_purple`（`0=blue`）；改用 0526 模型时是 `red_blue_gray_purple`。建议启动时把当前映射打印到日志。

### 3.3 解耦与状态管理

- `ShootMode` 从 `io/cboard.hpp` 移到 `io/command.hpp`；`aimer.hpp` 不再 include `cboard.hpp`。
- 新增 `Tracker::set_enemy_color()`：阵营变化时重置状态机和目标，避免继续跟踪换边前的颜色。

### 3.4 参数校验

- 所有新增或修改的 YAML 参数在启动时检查有限性和合法范围，非法时直接抛异常，不带病运行。
- 每个参数旁写中文注释：用途、单位、范围、对实时性或安全性的影响。

### 3.5 pitch 补偿支持两点插值

- `pitch_offset` 既接受标量（行为与原版相同），也接受 `near`/`far` 两点，按水平距离线性插值，超出范围取端点值。
- 校验：距离有限、`0 <= near.distance < far.distance`、偏移有限。
- 移植 `tests/aimer_pitch_offset_test.cpp` 和配套的 9 个 YAML。

### 3.6 离线测试

在 `BUILD_TESTING` 下加入不依赖硬件和 ROS 的 CTest：

- `aimer_spin_selection_test`
- `aimer_pitch_offset_test`
- `target_layout_test`
- `outpost_model_test`（如果移植了前哨站布局识别）
- `solver_image_size_test`（如果移植了 `set_image_size`）
- `handeye_transform_test`

## 4. P2 按需：有取舍的改动

| 改动 | 适用条件 | 代价 | 建议做法 |
| --- | --- | --- | --- |
| 大小装甲只按兵种判断 | 采用 2026 赛制（仅英雄大装甲） | 分类出错时 PnP 用错尺寸，失去长宽比兜底 | 按兵种决定类型，但保留长宽比作为一致性校验：明显矛盾时记 warning 或丢弃该目标 |
| 装甲属性表按当前赛制重写，删除平衡步兵和基地大装甲 | 同上 | 规则变化需要改代码 | 把赛制相关的表集中在一处，便于下赛季修改 |
| 前哨站高度布局识别 | 需要稳定打前哨站 | 三块装甲都观测到之前只打最后一块，首发变慢；依赖 0.102 m 高度间距假设；累积平均纠错慢 | 先移植并实测首发延迟；考虑对平均值加滑动窗口或设定最少观测次数 |
| `OUTPOST_RADIUS` 命名常量 | 总是值得 | 本项目把 0.2765 改成 0.275 | 采用命名常量，数值以实测为准 |
| `Solver::set_image_size` 按实际分辨率缩放内参 | 运行分辨率可能与标定分辨率不同 | 需要在 YAML 中增加标定分辨率字段 | 车载固定分辨率可以不做 |
| ONNX Runtime + CUDA 替换 OpenVINO | 运算平台是 NVIDIA | 失去 Intel 平台优化；v8/11 和打符需要另行移植 | Intel NUC 保留 OpenVINO；若移植，GPU 不可用时应启动失败，不要静默回退 CPU |
| 锁定 ONNX Runtime 版本 | 多台机器需要一致构建 | 部署门槛更高 | 团队统一环境时采用，并在 README 写明安装步骤 |
| 默认构建只保留自瞄链路 | 不需要打符、全向感知 | 打符等功能不可用 | 需要打符的兵种不要关闭 `auto_buff` |
| 远端链路（`AUVClient`、`custom_client`、`custom_client_adapter`、ROS 标定工具、时间戳同步） | 自瞄在远端 PC 上运行 | 多出编解码和无线传输的延迟与抖动，依赖增多 | 车载部署不需要移植 |

**无论哪种部署都建议吸收的思想**：失效时发布安全命令。车载版本可以在 `cboard` 输出路径上实现：检测、跟踪、解算失败，结果非有限，或输入超时时，发送不开火、不跟随旧目标的命令。

## 5. 不照搬

- **删除 `SIGTERM` 处理**：本项目的退化。被 `kill`、`systemctl stop` 或 launch 结束时不走析构，相机和线程可能释放不干净。
- **未配置就改变颜色默认顺序**：换回原版模型时红蓝会反过来，存在打己方的风险。
- **一次性大改动、混用 CRLF**：本项目大量文件处于未提交状态，且行尾混用。新版本应先加 `.gitattributes` 统一 LF，再小步提交。
- **未经评估移植 `yolov8pose`、MAVLink 等分支改动**：它们来自 `sp_vision_25-main`、`sp_vision_25-main-sentry`，本项目没有合入，需单独评估。

## 6. 推荐实施顺序

每一步单独提交，每步完成后构建并运行已有测试：

1. 从原版建立基线提交，加入 `.gitattributes`（`* text=auto eol=lf`）。
2. 搭建 CTest 骨架，先让构建和空测试跑通。
3. P0 第 1、2 项（小陀螺索引、绝对值），同时加入选板测试。
4. P0 第 3、4、5、6 项，各自一个提交。
5. P1：检测健壮性与写盘清理 → 类别映射与颜色配置 → 解耦与阵营重置 → 参数校验 → pitch 两点插值。
6. P2：按第 7 节的决策逐项移植，高风险项（检测逻辑、开火逻辑）必须同时补正常路径、无效输入和失效保护测试。

## 7. 验证清单

**离线**

- `cmake --build` 无新增警告，`ctest --output-on-failure` 全部通过。

**实车**

- 小陀螺：目标正转、反转、静止三种情况下选板正确，延迟分支切换正确。
- pitch 补偿：近、中、远三个距离实测落点，确认中间距离误差可接受。
- 颜色：红、蓝装甲各验证一次映射，切换阵营后跟踪器重置。
- 退出：`kill -TERM` 后相机、串口正常释放，下次可立即重新打开。
- 前哨站（若移植）：记录从发现到首发的时间。

## 8. 开工前需要确认

| 问题 | 影响的条目 |
| --- | --- |
| 车载部署还是远端部署 | 远端链路是否移植 |
| 运算平台是 Intel 还是 NVIDIA | OpenVINO 还是 ONNX Runtime + CUDA |
| 使用原版模型还是 0526 | 颜色顺序默认值、模型输入输出契约 |
| 是否保留打符、哨兵 MAVLink | 构建裁剪范围 |
| 是否需要打前哨站 | 前哨站布局识别是否移植 |
