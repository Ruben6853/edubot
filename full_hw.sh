#!/usr/bin/env bash
set -euo pipefail

# locate this script's directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

PCS="$SCRIPT_DIR/pos_control_hw.sh"
PP="$SCRIPT_DIR/pos_publisher.sh"

for f in "$PCS" "$PP"; do
  if [ ! -e "$f" ]; then
    echo "Required file $f not found" >&2
    exit 1
  fi
  if [ ! -x "$f" ]; then
    chmod +x "$f" || true
  fi
done

# start both in background and capture PIDs
"$PCS" >"$SCRIPT_DIR/pos_control_hw.log" 2>&1 &
PID1=$!
"$PP" >"$SCRIPT_DIR/pos_publisher.log" 2>&1 &
PID2=$!

trap 'echo "Stopping..."; kill -TERM "$PID1" "$PID2" 2>/dev/null || true' INT TERM

wait "$PID1"
STATUS1=$?
wait "$PID2"
STATUS2=$?

echo "pos_control_hw exit: $STATUS1"
echo "pos_publisher exit: $STATUS2"

if [ "$STATUS1" -ne 0 ] || [ "$STATUS2" -ne 0 ]; then
  exit 1
fi

exit 0
