#!/usr/bin/env bash
cd "$(dirname "$0")" || exit 1
mkdir -p logs

while true; do
  ./build/sentry_minipc configs/sentry.yaml >> logs/sentry_minipc.log 2>&1
  code=$?
  echo "[watchdog $(date '+%F %T')] 退出码=$code，2s 后重启" >> logs/sentry_minipc.log
  sleep 2
done
