# sp_vision_25 — 哨兵自瞄视觉（CDC / 新协议）

同济 SuperPower 哨兵视觉自瞄。视觉侧通过 USB CDC 与 rmpp 下位机通信。
本仓库为**新协议 A1/A2** 版本，完整协议见 [docs/protocol.md](docs/protocol.md)。

## 当前状态

- 新协议：视觉发 `0xA1`（17B）、收 `0xA2`（24B）；CRC 覆盖帧头，pitch 为**下正**。
- 实车（哨兵）在板子固件升级前暂时回退旧 `A5` 协议；本仓库为板子升级后使用的**目标版本**，新旧协议不兼容，需同步升级下位机固件。
- 上游项目说明见 [docs/upstream_readme.md](docs/upstream_readme.md)。

## 目录

| 目录 | 内容 |
|---|---|
| `src/` | 主程序（`sentry_minipc`） |
| `tasks/` | 识别、追踪、弹道、自瞄 |
| `io/` | 相机、串口、云台、minipc 等 |
| `configs/` | 运行配置（`sentry.yaml`） |
| `calibration/` | 标定工具 |
| `tools/` | 通用工具 |
| `tests/` | 测试 |
| `assets/` | 模型与少量运行资源 |
| `docs/` | 协议、部署文档 |

## 构建

依赖：CMake ≥ 3.16、C++17、OpenCV、Eigen3、yaml-cpp、OpenVINO（推理）、
海康 MVS SDK / 迈德威视 MVSDK（相机）。

> 厂商 SDK 的头文件与 `.so` **未包含**在本仓库（授权原因）。
> `io/hikrobot`、`io/mindvision` 仅保留项目封装（`.cpp/.hpp`）；请自行把 SDK 的
> `include/`、`lib/` 放回对应目录，否则相机相关目标无法编译。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)" --target sentry_minipc
```

## 运行

```bash
./build/sentry_minipc configs/sentry.yaml
```

串口默认 `/dev/c_board`（udev 软链到 `ttyACM*`）。

## 数据与排除项

- 包含：源码、配置、标定工具、模型（`assets/*.onnx|*.bin|*.xml`）。
- 未包含：`build/`、`logs/`、`records/`、`imgs/`、标定视频与帧（`*.avi`、`*_frames/`、`*_q.txt`）、截图、厂商 SDK 二进制/头文件。

## License

MIT，见 [LICENSE](LICENSE)（Copyright (c) 2025 TongjiSuperPower）。厂商 SDK 遵循各自许可。
