# 自瞄（sp_vision_25 / sentry）改动记录

> 部署目录：`~/Desktop/sp_vision_25_sentry_cdc_v2`（potential-jellysentry, 192.168.1.154）
> 下位机：rmpp（mc02, CherryUSB CDC, 115200, `/dev/c_board`）

## 2026-11-20 ~ 21

### 1. CDC 通信接口（新增）
- `io/minipc/protocol.hpp`、`io/minipc/minipc_io.{hpp,cpp}`
  - 固定长度帧，帧头 `0xA5 0x5A`（下行）/ `0xA5 0x5B`（上行），CRC-16/MODBUS（低字节先，范围不含帧头）。
  - 下行 21B：yaw/pitch/roll + is_red + bullet_speed。
  - 上行 18B：is_detect + yaw/pitch + is_fire + distance。
  - pitch 边界换算：线上“抬头正”，视觉内部“低头正”，上下行各取负一次。
  - 超时 100ms 断连；串口默认 `/dev/c_board`（可用 `com_port` 覆盖）。

### 2. 哨兵主程序
- `src/sentry_minipc.cpp`
  - 相机 → YOLO → Tracker → Aimer/Shooter → CDC 下发。
  - 1Hz 日志：`imu/q`、`armors/targets/state`、`detect/fire`、`cmd_yaw/pitch`、`dist`、`spin`、`sel(straight/spin)`、`pyaw(选中板 yaw)`、`type/name`、`cam(x,y,z)`、`aim(x,y,z)`、`aim_ok`。
  - 逐帧 CSV：`logs/aim.csv`（含 armor/aim/imu/cmd/dist/spin/sel/plate_yaw）。
  - Plotter UDP 目标可配置：`plotter_host` / `plotter_port`（默认 127.0.0.1:9870）。

### 3. 相机
- `io/hikrobot/hikrobot.{hpp,cpp}`、`io/camera.cpp`
  - 新增软件白平衡 `wb_gains: [b,g,r]`（raw Bayer 无硬件 WB，绿偏修正）。
  - 日志打印 wb_gains。
- 配置：`exposure_ms: 8`、`gain: 30`、`wb_gains: [1.13,0.85,1.05]`。
- 注意：帧率受 USB 带宽限制；换到 **USB3 口**后由 24Hz 提升到 ~115Hz。

### 4. 标定工具（新增/增强）
- `calibration/pattern.hpp`：棋盘格/圆点阵可配置（`pattern_type/cols/rows/spacing`）。
- `calibration/capture_minipc.cpp`：CDC 版采集（存 jpg + wxyz）。
- `calibration/record.cpp`：录视频 + 逐帧四元数（`every` 抽帧）。
- `calibration/extract_frames.cpp`：离线挑帧（检测成功 + 清晰度 + **云台静止 q 稳定**）。
- `calibration/calibrate_camera.cpp`、`calibrate_handeye.cpp`：新增 `--no-show` 批量模式；支持棋盘格。
- 标定板：8×11 内角点，方格 30mm。

### 5. Aimer
- `tasks/auto_aim/aimer.cpp`、`aimer.hpp`
  - 延时补偿由“只看自旋 vyaw”改成“**视在角速度**”：`omega_apparent = |中心方位角速度| + |vyaw|`，阈值 `apparent_speed_threshold`（默认 2.0 rad/s）。
  - 暴露 `select_mode`（none/straight/spin），供日志显示选板策略。
  - （`_v2` 原有：线性 `pitch_offset` near/far、`min_spin_speed`、前哨站 `|delta|` 最小 选板。）

### 6. 配置（configs/sentry.yaml）
- 内参：`camera_matrix`（fx≈1817，cx≈756，cy≈560）、`distort_coeffs`（重投影 ~0.39px）。
- 外参：`R_camera2gimbal`（理想）、`t_camera2gimbal`（相机相对俯仰中心 P：0.190, 0.050, 0.000 m）。
- `bullet_speed_default: 20`（下位机未给弹速时）。
- 瞄准：`yaw_offset`、`pitch_offset`、`comming_angle/leaving_angle`。
- 延时：`high_speed_delay_time: 0.026`、`low_speed_delay_time: 0.015`、`apparent_speed_threshold: 2.0`。
- `plotter_host: 192.168.1.230`、`plotter_port: 9870`。

### 7. 系统/部署
- udev：`/etc/udev/rules.d/99-rmpp-cdc.rules` → `1209:0001` interface 00 → `/dev/c_board`（0666）。
- 脚本：`start.sh` / `stop.sh` / `watchdog.sh` / `autostart.sh`。
- 关闭：`NetworkManager-wait-online`、`fwupd`（mask）；NoMachine `EnableAudio 0`（关麦克风）。

### 8. 另一台机（potential-i5-1235U, 192.168.1.155）
- 装依赖、OpenVINO 2024.6、MVS(~mvs412)、libceres-dev、intel-opencl-icd。
- 同步工程并编译 `sentry_minipc` 及标定工具。
- **注意**：该机标定需按本机相机/安装重做（当前配置来自 jellysentry）。

## 已知问题 / 待办
- 高速小陀螺：云台跟随带宽不足（`imu-cmd` 在 50–100°/s 约 -3.6°），主要受下位机云台环/带宽限制。
- 选板可能选到偏/侧棱的板（受击面小）：可改为“命中时刻真实受击面最大”的选择。
- 弹速目前默认 20；最好由下位机发真实弹速。
