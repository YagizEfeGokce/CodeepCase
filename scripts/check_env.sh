#!/usr/bin/env bash
# check_env.sh — kurulumu ve tüm bağımlılıkları doğrular (Go2 + G1).
set -e
cd "$(dirname "$0")/.."
ROOT="$PWD"
PY="$ROOT/.venv/bin/python"
ok=0
fail=0
chk() { if [ "$2" = "yes" ]; then
	echo "  [OK]   $1"
	ok=$((ok + 1))
else
	echo "  [FAIL] $1"
	fail=$((fail + 1))
fi; }

echo "=== Python venv & importlar ==="
[ -x "$PY" ] && chk ".venv/bin/python" yes || chk ".venv/bin/python" no
"$PY" - <<'PY' 2>/dev/null && chk "import {mujoco,cyclonedds,unitree_sdk2py,torch,onnxruntime}" yes || chk "import {mujoco,cyclonedds,unitree_sdk2py,torch,onnxruntime}" no
import mujoco, cyclonedds, unitree_sdk2py, torch, onnxruntime
PY

echo "=== Dış bağımlılıklar (external/, setup.sh ile klonlanır) ==="
for d in cyclonedds unitree_mujoco unitree_sdk2_python unitree-sim2real unitree_rl_gym; do
	[ -d "$ROOT/external/$d" ] && chk "external/$d" yes || chk "external/$d" no
done
[ -d "$ROOT/external/cyclonedds/install/lib" ] && chk "cyclonedds C kütüphanesi derlenmiş" yes || chk "cyclonedds C kütüphanesi derlenmiş" no

echo "=== Go2 ==="
[ -f "$ROOT/external/unitree_mujoco/unitree_robots/go2/scene_clean.xml" ] && chk "Go2 scene_clean.xml kurulu" yes || chk "Go2 scene_clean.xml kurulu (setup.sh step 5)" no
[ -f "$ROOT/external/unitree_mujoco/unitree_robots/go2/scene_obstacle.xml" ] && chk "Go2 scene_obstacle.xml kurulu" yes || chk "Go2 scene_obstacle.xml kurulu (setup.sh step 5)" no
[ -f "$ROOT/external/unitree_mujoco/unitree_robots/go2/go2_rangefinder.xml" ] && chk "Go2 go2_rangefinder.xml kurulu (Gate E --rf)" yes || chk "Go2 go2_rangefinder.xml kurulu (setup.sh step 5)" no
[ -f "$ROOT/external/unitree_mujoco/unitree_robots/go2/scene_obstacle_rf.xml" ] && chk "Go2 scene_obstacle_rf.xml kurulu (Gate E --rf)" yes || chk "Go2 scene_obstacle_rf.xml kurulu (setup.sh step 5)" no
[ -f "$ROOT/external/unitree-sim2real/RL_policy_runner/policies/one_for_all/all_gait_23Dec2025.pt" ] && chk "Go2 RL trot policy" yes || chk "Go2 RL trot policy" no
[ -f "$ROOT/external/diasAiMaster_go2_velocity_flat/policy.onnx" ] && chk "Go2 ONNX vy policy (diasAiMaster, Gate D/E --onnx)" yes || chk "Go2 ONNX vy policy (setup.sh step 6 fetch)" no

echo "=== G1 (bonus) ==="
[ -f "$ROOT/external/unitree_rl_gym/resources/robots/g1_description/scene.xml" ] && chk "G1 MJCF scene" yes || chk "G1 MJCF scene" no
[ -f "$ROOT/external/unitree_rl_gym/deploy/pre_train/g1/motion.pt" ] && chk "G1 motion policy" yes || chk "G1 motion policy" no

echo
echo "Sonuç: $ok OK, $fail FAIL"
[ "$fail" -eq 0 ] && echo "✓ Ortam hazır. 'bash run.sh b' ile başlayabilirsiniz." || {
	echo "✗ Eksik var — 'bash scripts/setup.sh' çalıştırın."
	exit 1
}
