#!/usr/bin/env bash
# run.sh — tek-komutla simülatörü başlat + gate'i çalıştır + kapat.
# Kullanım:
#   bash run.sh b                 # Gate B: dik dur
#   bash run.sh c --vx 0.3         # Gate C: ileri yürü (ek argüman gate'e iletilir)
#   bash run.sh d                  # Gate D: hedefe git (straight_walk)
#   bash run.sh e                  # Gate E: engelden kaçın
#   bash run.sh f                  # Gate F+: 4 waypoint + engel (tek run)
#   bash run.sh f-waypoints        # Gate F (sadece waypoint, engelsiz)
#   bash run.sh g1                 # Gate G: G1 humanoid (kendi viewer'ı; sim gerekmez)
#   bash run.sh watch-stand        # Go2 dik durmayı canlı izle
#   bash run.sh watch-walk         # Go2 yürüyüşü canlı izle
#   bash run.sh sim [clean|obstacle]  # sadece simülatörü başlat
set -e
cd "$(dirname "$0")"
ROOT="$PWD"
PY="$ROOT/.venv/bin/python"
SIMDIR="$ROOT/external/unitree_mujoco/simulate_python"
PIDF=/tmp/codeep_sim.pid
LOG=/tmp/codeep_sim.log
if [ -z "${DISPLAY:-}" ] && [ "${HEADLESS:-0}" != "1" ]; then export DISPLAY=:1; fi

need_venv() {
	[ -x "$PY" ] || {
		echo "[run] .venv yok — önce 'bash scripts/setup.sh' çalıştırın."
		exit 1
	}
}
start_sim() { # $1 = clean|obstacle
	[ -d "$SIMDIR" ] || {
		echo "[run] external/unitree_mujoco yok — 'bash scripts/setup.sh'."
		exit 1
	}
	[ -f "$PIDF" ] && kill "$(cat "$PIDF")" 2>/dev/null
	sleep 1
	bash scripts/use_scene.sh "$1"
	if [ "${HEADLESS:-0}" = "1" ] || [ -z "${DISPLAY:-}" ]; then
		(SDL_VIDEODRIVER=dummy "$PY" scripts/sim_headless.py --duration 180) >"$LOG" 2>&1 &
		echo "[run] headless sim başlatılıyor (sahne=$1, pid=$!)... 6 sn bekleniyor"
	else
		(cd "$SIMDIR" && "$PY" unitree_mujoco.py) >"$LOG" 2>&1 &
		echo "[run] sim başlatılıyor (sahne=$1, pid=$!)... 6 sn bekleniyor"
	fi
	echo $! >"$PIDF"
	sleep 6
	kill -0 "$(cat "$PIDF")" 2>/dev/null || {
		echo "[run] sim çöktü. Log:"
		tail -20 "$LOG"
		exit 1
	}
	echo "[run] sim hazır. (log: tail -f $LOG)"
}
stop_sim() {
	[ -f "$PIDF" ] && kill "$(cat "$PIDF")" 2>/dev/null
	rm -f "$PIDF"
}
trap stop_sim EXIT

gate="$1"
shift 2>/dev/null || true
need_venv
case "$gate" in
b)
	start_sim clean
	"$PY" gates/gate_b_stand.py "$@"
	;;
c)
	start_sim clean
	"$PY" gates/gate_c_rl.py "$@"
	;;
d | straight)
	start_sim clean
	"$PY" gates/straight_walk.py "$@"
	;;
e)
	start_sim obstacle
	"$PY" gates/gate_e_obstacle.py "$@"
	;;
f)
	start_sim obstacle
	"$PY" gates/gate_f_combined.py "$@"
	;;
f-waypoints)
	start_sim clean
	"$PY" gates/gate_f_waypoints.py "$@"
	;;
g1) "$PY" gates/gate_g_g1.py "$@" ;; # standalone, sim gerekmez
watch-stand)
	start_sim clean
	"$PY" scripts/stand_watch.py "$@"
	;;
watch-walk)
	start_sim clean
	"$PY" scripts/walk_watch.py "$@"
	;;
sim)
	bash scripts/use_scene.sh "${1:-clean}"
	if [ "${HEADLESS:-0}" = "1" ] || [ -z "${DISPLAY:-}" ]; then
		SDL_VIDEODRIVER=dummy "$PY" scripts/sim_headless.py --duration 180
	else
		(cd "$SIMDIR" && "$PY" unitree_mujoco.py)
	fi
	;;
*)
	echo "Kullanım: bash run.sh b|c|d|e|f|f-waypoints|g1|watch-stand|watch-walk|sim [clean|obstacle] [-- gate-args]"
	exit 1
	;;
esac
