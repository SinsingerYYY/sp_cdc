# 车载部署注意事项

适用范围：本仓库（基于 sp_vision_25 原版改造）在 Intel NUC + OpenVINO 上的车载部署。
依据：截至 2026-10-31 的代码（`687fdf7`）。离线 CTest 7/7 通过，**尚未实车验证**。

## 1. 上车前必须处理（阻塞项）

| # | 问题 | 后果 | 处理 |
| --- | --- | --- | --- |
| 1 | 不确定车上用的是原版 `assets/yolov5.xml`、0526 还是深大模型 | 颜色顺序不对时红蓝反转，**可能打己方** | 所有配置默认 `yolov5_color_order: blue_red_gray_purple`（对应原版模型）。上车后按第 4 节第 2 步实测确认 |
| 2 | `autostart.sh` 调用的 `watchdog.sh` 不在仓库里，且路径写死为 `~/Desktop/sp_vision_25/` | 开机自启直接失败 | 补上 `watchdog.sh`，或把 `autostart.sh` 改成直接启动主程序；路径改成车上实际目录 |
| 3 | `configs/example.yaml` 缺 `min_spin_speed`、`decision_speed` | 用它启动会在 Aimer 构造时抛异常退出 | 不要用 `example.yaml` 上车；以 `standard3.yaml` / `standard4.yaml` / `sentry.yaml` 等为模板 |
| 4 | 标定分辨率缩放（`Solver::set_image_size`）目前只在测试中调用，主程序没有接入 | 运行分辨率与标定分辨率不同时 PnP 距离错误 | 保证相机输出分辨率与标定时一致；如需不同分辨率，先在主程序接入 `set_image_size` |

## 2. 编译

```bash
cmake -B build -DBUILD_TESTING=ON
make -C build -j$(nproc)
cd build && ctest --output-on-failure   # 必须 7/7 通过
```

- 依赖：海康 MVS 或迈德威视 SDK、OpenVINO、Ceres、OpenCV、Eigen、yaml-cpp、fmt、spdlog。
- **fmt 与 spdlog 版本必须匹配**。开发机上一个构建目录出现过 `'basic_runtime' is not a member of 'fmt'` 和 `fmt::v8::format_error` 链接错误，原因是系统 spdlog 与另一版本的 fmt 混用。车载 NUC 上建议只装 apt 自带的一套，不要混用源码安装的 fmt。
- 编译不应出现新的 warning（OpenCV 的 deprecated 警告除外）。

## 3. 配置文件检查

启动时会校验参数，非法时直接抛异常退出，不会带病运行。上车前逐项确认所用 yaml：

| 字段 | 要求 | 说明 |
| --- | --- | --- |
| `enemy_color` | `red` / `blue` | 比赛前按抽签结果确认；运行中阵营变化会重置跟踪器 |
| `yolo_name` | `yolov5` | 当前只验证过 YOLOv5 路径 |
| `yolov5_color_order` | red/blue/gray/purple 的一个排列 | 必须与模型一致，见第 1 节第 1 项 |
| `device` | 车载用 `GPU` | 用 iGPU 推理；`CPU` 只用于开发机 |
| `use_roi` / `roi` | `use_roi: true` 时 x、y ≥ 0，宽高为 -1 或正数 | 越界会在构造时报错 |
| `min_spin_speed` | 有限且 ≥ 0，单位 rad/s | 角速度绝对值不超过此值视为不转 |
| `decision_speed` | 有限，单位 rad/s | 角速度绝对值超过此值用高速延迟 |
| `pitch_offset` | 标量（度），或 `near`/`far` 两点 | 两点格式要求 `0 <= near.distance < far.distance`，超出范围取端点值 |
| 相机内参、手眼外参 | 与当前相机、镜头、安装位置一致 | 换镜头或拆装云台后必须重新标定 |

## 4. 上车验证顺序

每一步通过后再进行下一步，结果记录在调试日志里。

1. **启动检查**：启动后日志无参数异常；确认 `[YOLOV5] color id mapping: 0=... 1=...` 与预期一致。
2. **颜色**：分别放红、蓝装甲，确认识别颜色正确；运行中切换 `enemy_color`，确认跟踪器重置、不再跟旧目标。
   - 若红色被识别成蓝色，说明模型是 0526 一类，改为 `yolov5_color_order: red_blue_gray_purple`。
3. **装甲识别**：打开 debug，看是否频繁出现 `see strange armor`。
   - 小装甲长宽比上限为 3.0（`SMALL_ARMOR_MAX_RATIO`，`tasks/auto_aim/armor.hpp`），正视约 2.4，余量不大。若出现“看得见但不跟”，先查这里。
4. **pitch 补偿**：静止目标在近、中、远三个距离各打一组，确认落点；两点插值时重点看中间距离。
5. **小陀螺**：目标正转、反转、静止三种情况，确认选板正确、高低速延迟切换正确。
6. **前哨站**：记录从发现到首发的时间。
   - 布局需三块装甲各观测满 3 次（`OUTPOST_MIN_HEIGHT_OBSERVATIONS`）才完成，之前只打当前看到的那块。太慢可改为 2。
   - `OUTPOST_RADIUS` 当前为 0.2765 m，需实测后统一（测试文字写的是 0.275）。
7. **退出与重启**：`kill -TERM <pid>` 后立即重启，确认相机、串口能正常重新打开。
8. **失效保护**：遮挡相机、拔掉目标，确认云台不再跟随旧目标、不开火。

## 5. 开机自启

1. 处理第 1 节第 2 项（`watchdog.sh` 和路径）。
2. `chmod +x autostart.sh`，安装 `screen`。
3. 在 `~/.config/autostart/` 下创建 `.desktop` 文件，`Exec=` 指向 `autostart.sh` 的绝对路径。
4. 日志写在 `logs/`，确认目录存在且磁盘空间足够；定期清理。
5. 重启 NUC，确认程序自动起来、日志正常写入。

## 6. 赛场注意事项

- 每场前确认 `enemy_color`，以及所用配置文件就是调试时验证过的那份。
- 不在赛场临时改代码；只改 yaml 参数，改完先看启动日志无异常。
- 保留每场日志，赛后用于复现问题。
- 换模型（包括深大开源模型）前，必须重新核对颜色顺序、类别映射（num_id 8 当前视为非装甲）、角点顺序和置信度是否已做 sigmoid，并重新走第 4 节。

## 7. 已知限制

- 离线测试只覆盖选板、pitch 插值、装甲布局、前哨站、内参缩放、安全命令和手眼变换，不覆盖相机、串口、推理和实时性能。
- 小装甲长宽比阈值、前哨站观测次数、前哨站半径均未实测。
- 基地顶部装甲（num_id 8）当前被丢弃。
