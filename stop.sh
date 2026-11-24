#!/usr/bin/env bash
cd "$(dirname "$0")" || exit 1

if pkill -x sentry_minipc; then
  echo "[stop] sentry_minipc 已停止"
else
  echo "[stop] sentry_minipc 未运行"
fi

pkill -f "watchdog.sh" 2>/dev/null
rm -f logs/sentry_minipc.pid
