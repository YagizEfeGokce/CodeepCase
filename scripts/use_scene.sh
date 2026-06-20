#!/usr/bin/env bash
# unitree_mujoco simülatörünün sahne dosyasını değiştir (clean / obstacle).
# Kullanım: bash scripts/use_scene.sh clean   |   bash scripts/use_scene.sh obstacle
set -e
cd "$(dirname "$0")/.."
CFG="external/unitree_mujoco/simulate_python/config.py"
case "${1:-clean}" in
clean) SCENE="scene_clean.xml" ;;
obstacle) SCENE="scene_obstacle.xml" ;;
*)
	echo "kullanım: $0 clean|obstacle"
	exit 1
	;;
esac
python3 - "$CFG" "$SCENE" <<'PY'
import sys, re
cfg, scene = sys.argv[1], sys.argv[2]
s = open(cfg).read()
s = re.sub(r'ROBOT_SCENE = ".*?"', f'ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/{scene}"', s)
s = re.sub(r'USE_JOYSTICK = \d+', 'USE_JOYSTICK = 0', s)
open(cfg, "w").write(s)
print(f"[use_scene] ROBOT_SCENE -> {scene}, USE_JOYSTICK -> 0")
PY
