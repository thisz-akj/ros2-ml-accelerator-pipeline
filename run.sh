#!/bin/bash

set -e

if [ -z "$1" ]; then
  echo "Usage: ./run.sh <package_name> [--fg|--foreground]"
  exit 1
fi

PACKAGE_NAME=$1
FOREGROUND=false

if [ "$2" == "--fg" ] || [ "$2" == "--foreground" ]; then
  FOREGROUND=true
fi

echo "[INFO] Setting LD_LIBRARY_PATH..."
export LD_LIBRARY_PATH="$(pwd)/build/${PACKAGE_NAME}:${LD_LIBRARY_PATH}"

echo "[INFO] Sourcing install/setup.bash..."
source install/setup.bash

echo "[INFO] Setting VDP_ROS_PACKAGE_DIR..."
export VDP_ROS_PACKAGE_DIR="$(pwd)/packages/${PACKAGE_NAME}"

PARAM_FILE="packages/${PACKAGE_NAME}/params/${PACKAGE_NAME}.yaml"

if [ ! -f "$PARAM_FILE" ]; then
  echo "[ERROR] Parameter file not found: $PARAM_FILE"
  exit 1
fi

APP_BIN="./install/lib/${PACKAGE_NAME}/${PACKAGE_NAME}_app"

if $FOREGROUND; then
  echo "[INFO] Running in foreground:"
  echo "--------------------------------------------------"
  exec "$APP_BIN" "$PARAM_FILE"
else
  echo "[INFO] Launching in background with nohup:"
  nohup "$APP_BIN" "$PARAM_FILE" > ~/${PACKAGE_NAME}_run.log 2>&1 &
  echo "[INFO] Process started in background. Logs: ~/${PACKAGE_NAME}_run.log"
  echo "[INFO] PID: $!"
fi