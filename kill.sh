#!/bin/bash

set -e

if [ -z "$1" ]; then
  echo "[ERROR] Package name required."
  echo "Usage: ./kill.sh <package_name>"
  exit 1
fi

PACKAGE_NAME="$1"
APP_NAME="${PACKAGE_NAME}_app"
PROCESS_PATTERN="./install/lib/${PACKAGE_NAME}/${APP_NAME}"

PIDS=$(pgrep -f "$PROCESS_PATTERN" || true)

if [ -z "$PIDS" ]; then
    echo "[INFO] ${APP_NAME} is not running."
    exit 0
fi

echo "[INFO] ${APP_NAME} running with PID(s): $PIDS"

for PID in $PIDS; do
    echo "[INFO] Sending SIGUSR2 to PID $PID"
    kill -USR2 "$PID"
done
