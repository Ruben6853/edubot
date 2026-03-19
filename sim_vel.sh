#!/usr/bin/env bash
set -euo pipefail

# locate this script's directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

PCS="$SCRIPT_DIR/vel_control_sim.sh"
PP="$SCRIPT_DIR/vel_publisher.sh"

for f in "$PCS" "$PP"; do
  if [ ! -e "$f" ]; then
    echo "Required file $f not found" >&2
    exit 1
  fi
  if [ ! -x "$f" ]; then
    chmod +x "$f" || true
  fi
done

# start both in background and print output in this same terminal
"$PCS" &
PID1=$!
"$PP" &
PID2=$!

trap 'echo "Stopping..."; kill -TERM "$PID1" "$PID2" 2>/dev/null || true' INT TERM

wait "$PID1"
STATUS1=$?
wait "$PID2"
STATUS2=$?

echo "vel_control_sim exit: $STATUS1"
echo "vel_publisher exit: $STATUS2"

if [ "$STATUS1" -ne 0 ] || [ "$STATUS2" -ne 0 ]; then
  exit 1
fi

exit 0
