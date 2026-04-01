#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# run_demo.sh  –  Tasks 2.1 + 2.3 demo (simulatie of hardware)
#
# Gebruik (WSL terminal, vanuit Wessel/ of repo-root):
#   bash run_demo.sh              # simulatie
#   bash run_demo.sh hw           # echte robot
#   bash run_demo.sh --no-build   # bouwen overslaan
#   bash run_demo.sh hw --no-build
# ─────────────────────────────────────────────────────────────────────────────
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_WS="$REPO_DIR/ros_ws"

# ── Argumenten ────────────────────────────────────────────────────────────────
MODE="sim"
NO_BUILD=0
for arg in "$@"; do
    case "$arg" in
        hw)         MODE="hw" ;;
        --no-build) NO_BUILD=1 ;;
        -h|--help)  echo "Gebruik: $0 [hw] [--no-build]"; exit 0 ;;
        *)          echo "[WARN] Onbekend argument: $arg" ;;
    esac
done

# ── Kleuren ───────────────────────────────────────────────────────────────────
G='\033[1;32m'; Y='\033[1;33m'; R='\033[1;31m'; B='\033[1;34m'; N='\033[0m'
info()  { echo -e "${G}[INFO]${N}  $*"; }
warn()  { echo -e "${Y}[WARN]${N}  $*"; }
error() { echo -e "${R}[ERROR]${N} $*"; exit 1; }
step()  { echo -e "\n${B}══ $* ${N}"; }

# ── 1. ROS sourcen ────────────────────────────────────────────────────────────
step "1/4  ROS sourcen"
ROS_SETUP=""
for candidate in /opt/ros/jazzy/setup.bash /opt/ros/humble/setup.bash; do
    [[ -f "$candidate" ]] && ROS_SETUP="$candidate" && break
done
[[ -z "$ROS_SETUP" ]] && error "Geen ROS gevonden in /opt/ros/."
source "$ROS_SETUP"
info "$ROS_SETUP geladen"

# ── 2. Bouwen ─────────────────────────────────────────────────────────────────
step "2/4  Bouwen"
cd "$ROS_WS"
if [[ $NO_BUILD -eq 1 ]]; then
    warn "--no-build: bouwen overgeslagen."
else
    info "controllers_new bouwen..."
    colcon build --symlink-install \
        --packages-select controllers_new python_controllers \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        2>&1 | grep -E '(Starting|Finished|Failed|error:|warning:|Summary)'
fi
source "$ROS_WS/install/setup.bash"
info "Workspace gesourced."

# ── 3. Stale processen stoppen ────────────────────────────────────────────────
step "3/4  $MODE starten"
pkill -f "ros2 launch lerobot"         2>/dev/null || true
pkill -f "ros2 run controllers_new"    2>/dev/null || true
pkill -f "ros2 run python_controllers" 2>/dev/null || true
sleep 1

LAUNCH_PID=""
cleanup() {
    echo ""
    info "Afsluiten..."
    [[ -n "$LAUNCH_PID" ]] && kill "$LAUNCH_PID" 2>/dev/null || true
    pkill -f "ros2 launch lerobot" 2>/dev/null || true
    info "Klaar."
}
trap cleanup EXIT INT TERM

if [[ "$MODE" == "hw" ]]; then
    LAUNCH_FILE="hw_position.launch.py"
    warn "Hardware-modus — zorg dat de robot aan staat en USB aangesloten is!"
    echo -n "  Druk ENTER om te starten, Ctrl+C om te annuleren... "
    read -r
else
    LAUNCH_FILE="sim_position.launch.py"
    info "Simulatie-modus."
fi

ros2 launch lerobot "$LAUNCH_FILE" &
LAUNCH_PID=$!
info "Gestart: $LAUNCH_FILE (pid $LAUNCH_PID)"

# ── 4. Wachten op robot, dan controller starten ───────────────────────────────
step "4/4  Controller starten"
info "Wachten op /joint_states (max 30 s)..."
ELAPSED=0
until ros2 topic list 2>/dev/null | grep -q "/joint_states"; do
    sleep 1; ELAPSED=$((ELAPSED + 1))
    [[ $ELAPSED -ge 30 ]] && error "Timeout bij wachten op /joint_states."
done
info "Robot klaar na ${ELAPSED}s. 2 s wachten..."
sleep 2

echo -e "\n${Y}Volgorde:${N}"
echo "  • Home (5 s)"
echo "  • Pose  I  [0.2, 0.2, 0.2]    → hold 2 s → home"
echo "  • Pose II  [0.2, 0.1, 0.4]    → hold 2 s → home"
echo "  • Pose  V  [0.0, 0.05, 0.45]  → hold 2 s → home"
echo "  • Cirkelbaan (60 punten, 12 s)"
echo ""

ros2 run controllers_new pos_traj
