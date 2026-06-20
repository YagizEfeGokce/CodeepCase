#!/usr/bin/env bash
# unitree_mujoco simülatörünün sahne dosyasını değiştir (clean / obstacle).
# Kullanım: bash scripts/use_scene.sh clean   |   bash scripts/use_scene.sh obstacle
set -e
cd "$(dirname "$0")/.."
CFG="external/unitree_mujoco/simulate_python/config.py"
case "${1:-clean}" in
  clean)    SCENE="scene_clean.xml" ;;
  obstacle) SCENE="scene_obstacle.xml" ;;
  *) echo "kullanım: $0 clean|obstacle"; exit 1 ;;
esac
python3 - "$CFG" "$SCENE" <<'PY'
import sys
cfg, scene = sys.argv[1], sys.argv[2]
lines = open(cfg).read().splitlines()
out = []
for ln in lines:
    s = ln.lstrip()
    indent = ln[:len(ln) - len(s)]
    if s.startswith("ROBOT_SCENE"):
        out.append(f'{indent}ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/{scene}"  # {scene}')
    elif s.startswith("USE_JOYSTICK"):
        out.append(f'{indent}USE_JOYSTICK = 0  # no gamepad')
    else:
        out.append(ln)
open(cfg, "w").write("\n".join(out) + "\n")
print(f"[use_scene] ROBOT_SCENE -> {scene}, USE_JOYSTICK -> 0")
PY