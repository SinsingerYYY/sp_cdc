#!/usr/bin/env bash
set -u
cd "$(dirname "$0")" || exit 1

BIN=./build/sentry_minipc
CONFIG=configs/sentry.yaml
LOG_DIR=logs
mkdir -p "$LOG_DIR"

[ -x "$BIN" ] || { echo "[start] 缺少 $BIN，请先编译"; exit 1; }
[ -e /dev/c_board ] || echo "[start] 警告: /dev/c_board 不存在（下位机 CDC 未连接）"
lsusb 2>/dev/null | grep -qi 2bdf || echo "[start] 警告: 未检测到海康相机 (2bdf)"

PIDFILE="$LOG_DIR/sentry_minipc.pid"
if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
  echo "[start] 已在运行 pid=$(cat "$PIDFILE")"
  exit 0
fi

nohup "$BIN" "$CONFIG" >> "$LOG_DIR/sentry_minipc.log" 2>&1 &
pid=$!
echo "$pid" > "$PIDFILE"
echo "[start] pid=$pid，日志: $LOG_DIR/sentry_minipc.log"
