# AI 对自瞄（sp_vision_25 / sentry）的改动记录

> 记录日期：2026-11-25。记录时 `.154`/`.155` 均离线，部分内容来自会话、`.154` git 历史与本地副本。
> 所有提交使用仓库 git 身份 `SingerYYY`，git 历史本身不区分 AI / 人。
> 本文件只记录 **AI 在本次协作中做的改动**，不替代部署/调参说明。

## 0. 一句话结论

AI 主要做了：CDC 通信（旧 A5 → 新 A1/A2）、自瞄/云台算法实验与回退、手眼外参重标定、
偏移/曝光调参、延迟与带宽分析、相机/系统部署工具、以及仓库整理与公开。
**未做**：下位机固件烧录、实弹命中/场地验证；因此“是否真的更准”没有命中数据佐证。

## 1. 时间线

### 2026-11-20 ~ 21（初始改造，见 `CHANGELOG.md`）
- 新增 `io/minipc`（旧协议 A5 5A/5B/5C，CRC 不含帧头）。
- 新增 `src/sentry_minipc.cpp`：相机 → YOLO → Tracker → Aimer/Shooter → CDC；1Hz 日志 + 逐帧 `logs/aim.csv` + UDP plotter。
- 相机：软件白平衡 `wb_gains`；USB2 → USB3 后帧率 24Hz → ~104–115Hz。
- 标定工具：`capture_minipc`、`record`、`extract_frames`、`calibrate_*`（`--no-show`、棋盘格）。
- Aimer：延时补偿改为“视在角速度”`omega_apparent = |中心方位角速度| + |vyaw|`。
- 配置：内参（fx≈1817, cx≈756, cy≈560, 重投影 ~0.39px）、外参、弹速默认 20、延时 .026/.015。
- 系统：udev `99-rmpp-cdc.rules`、`start/stop/watchdog/autostart`；`.155` 装依赖/OpenVINO/MVS 并编译。

### 2026-11-23（标定 / 外参）
- handeye3–10 采集与分析：
  - 曝光/增益问题：`gain 30` 越界不生效；4ms 偏暗、24ms/gain16 亮度正常（mean≈210）。
  - handeye7 姿态过窄（yaw 16°/pitch 3.4°）→ 解出 17° 垃圾；`assets/img_with_q` 旧 40 帧同样垃圾。
  - **采用 handeye9**：17 帧，姿态 yaw 25.8°/pitch 18.8°；相对理想 R 偏差 yaw −0.34° / pitch +2.95° / roll −0.29°；
    `t = [0.19272, 0.06053, 0.00981]`；分半一致（t_x 0.1917/0.1927、t_y 0.0661/0.0674；pitch 2.45/3.73）。
- 认识：**相机与炮口同高**时，相机缺 `t_z=0.094` 与“弹从原点发”少 0.094 **互相抵消**；
  不可只补相机 `t_z`（会让 1m 处打高 ~5.4°）。

### 2026-11-24（调参 / 相位 / 协议）
- Aimer 实验：隔项差、超前、`yaw_scale`、muzzle 补偿、MPC 接入；实测高速不理想（MPC `fire_thresh` 过严等）
  → **整体回退 baseline**（`a92bbdc`），`use_mpc: false`。
- 偏移调参（`.154` 提交）：`cc82e3b` yaw 0→−0.3 → `decdaff` pitch −0.5 → `f1f9a46` pitch −0.5 →
  `72df8dc` pitch −0.7 → `79bdfbd` pitch −1.0 → `1997121` yaw −0.7；左右 offset 归 0。
- 相位/延迟：
  - 逐帧 104Hz 停稳段：`imu−cmd pitch` 均值 +0.11°、std 1.6 → 静态云台跟随正常；
    之前 1Hz 看到的“−2°、随 yaw 变化”是稀疏采样假象。
  - ~75rpm（1.25Hz）：`phase(imu−cmd) ≈ −70~−80°`、gain ≈ 0.7–1.0 → **等效端到端延迟 ≈160ms**（延迟主导）。
  - 需求：`φ_max=57.3°·(w/r)`、`τ_max=(w/r)/ω`；`w/r=0.3` 时 60rpm→17.2°/48ms、90rpm→32ms、120rpm→24ms。
    输出 `/home/singer/响应指标.md`；`live_phase.py` 实时相位图。
- 新协议改造：
  - 旧：A5 + 5A/5B/5C，CRC 不含帧头，pitch 线上“上正”，下行欧拉角。
  - 新：单字节 A1/A2/A3/A4，CRC **含帧头**，pitch **下正**，A2 直接给四元数 `x,y,z,w`。
  - 视觉实现（A1 17B / A2 24B，A4 跳过、失步丢 1 字节）并编译；A2 参考帧 CRC 校验通过。

### 2026-11-25（协议验证 / 同步 / 仓库）
- 串口原始帧实测：板子仍发 **旧 A5 5A 21B @192Hz**；384/384 旧 CRC 通过，新 A2 **0 帧**通过。
  → 视觉新协议收不到（`conn=false`）。
- 决策：**只保留新协议，等板子更新**；但为恢复实车，`.154` 已 `git checkout` 回退旧 A5 并重编，`conn=true`。
- `.155` 切到新协议（`io/minipc` 三文件与 `.154` 新代码 md5 一致）并重编；该机无板子。
- `.155`：NoMachine 9.1.24（Xorg、GDM 自动登录）、关 WiFi 省电、`stream_udp`/`recv_stream.py`、
  海康 udev/USB `authorized` 复位；无板时用虚拟串口 `/dev/pts/1` 跑“检测-only”（`conn=false`）。
- 仓库：`sp_cdc`（公开）推协议文档 + 全量源码（`ee46a84`、`d0d5c0e`）。
- 去掉白平衡：删除软件 `wb_gains`（`io/hikrobot` 逐通道乘法、`io/camera` 读取、`configs/sentry.yaml`），硬件 `BalanceWhiteAuto` 改 `OFF`；**未编译/未上车验证**（设备离线）。

## 2. 当前最终参数（`.154`，已提交到 `1997121`）

| 项 | 值 |
|---|---|
| 协议 | **旧 A5**（实车运行）；新 A1/A2 在 `.155` 与 `sp_cdc` |
| `exposure_ms` / `gain` | 8 / 10 |
| `yaw_offset` / `pitch_offset` | −0.7 / −1.0 |
| `left/right_yaw_offset` | 0 / 0 |
| `R_camera2gimbal` / `t_camera2gimbal` | handeye9 / `[0.19272, 0.06053, 0.00981]` |
| `bullet_speed_default` | 20 m/s（板子旧帧送 0，视觉回退） |
| `high/low_speed_delay_time` | 0.026 / 0.015 s |
| `apparent_speed_threshold` | 2.0 rad/s |
| `use_mpc` / `auto_fire` | false / true |
| 串口 | `/dev/c_board` → `ttyACM0` |
| 相机 | hikrobot（USB3） |
| 白平衡 | **已去掉**（硬件 `BalanceWhiteAuto=OFF`，无软件增益） |

`.155` 未同步上述配置：仍 `exposure 4 / gain 16`、`yaw −0.8 / pitch −1`、左右 −1/−0.6、旧外参 `t=[0.1309,0.00385,0.0941]`。

## 3. 已回退 / 未采用（重要）

| 内容 | 结果 | 原因 |
|---|---|---|
| 隔项差 / 超前 / `yaw_scale` / muzzle / MPC | 回退 baseline（`a92bbdc`） | 实测不佳；MPC 高速不发射 |
| 新协议 A1/A2 在 `.154` 常驻 | 暂时回退旧 A5 | 板子固件仍旧 A5，`conn=false` |
| handeye3/7 等外参 | 未采用 | 姿态窄，解出 17° 垃圾 |
| 单独补相机 `t_z=0.094` | 未采用 | 与“炮口同高”抵消，单独补会打高 |
| 软件白平衡 `wb_gains` | 去掉 | 固定增益有 CPU/延迟成本；与硬件自动 WB 叠加 |
| `.154` 上真实自瞄的新协议验证 | 未完成 | 板子未升级 |

## 4. 记录来源与可信度

| 内容 | 来源 | 可信度 |
|---|---|---|
| 11/20–21 初始改造 | `CHANGELOG.md` | 高 |
| 偏移/协议/回退提交 | `.154` git log（`cc82e3b`…`1997121`、`a92bbdc` 等） | 高（部分早期 hash 未复核） |
| 新旧协议判定 | 原始串口 384 帧 CRC 分析 | 高 |
| 延迟/相位 | `响应指标.md`、逐帧 `aim.csv`、`analyze_phase.py` | 中（离线/近似） |
| handeye9 | 采集分析 + 分半一致性 | 中（未实弹验证） |
| `.155` 部署 | 会话/本机记录 | 中（设备当前离线） |
| 仓库推送 | `sp_cdc` commits `ee46a84`、`d0d5c0e` | 高 |

> 缺口：11/23–25 的细节此前只散落在 git 与对话中；本文件为首次综合。
> 仍未记录：实弹/场地命中数据（不存在），下位机固件改动（未做）。

## 5. 待办

1. 下位机固件升级到 A1/A2；板子发 A2 后 `.154` 切回新协议（无需重启，`read_thread` 自动恢复）。
2. `.155` 若作为自瞄车：同步 `.154` 的外参/offset，并把曝光改回 8ms。
3. 高转速：`τ_eq≈160ms` 远超 60rpm 的 `τ_max≈48ms`；优先降曝光/推理/通信/下位机环延迟。
4. 命中验证：需要实弹/落点数据，才能确认 offset 与外参。
5. `.155` 上线后把 `stream_udp` 等新增工具补进本仓库。
