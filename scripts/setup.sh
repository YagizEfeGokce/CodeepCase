#!/usr/bin/env bash
# CodeepV1 kurulumu: venv + dış bağımlılıkları klonla + CycloneDDS derle + pip kur.
# sudo gerektirmez. Ubuntu 24.04 + Python 3.12'de test edildi.
set -e
cd "$(dirname "$0")/.."
ROOT="$PWD"

echo "=== 1) venv (--without-pip; ensurepip Debian'da yok) ==="
python3 -m venv --without-pip .venv
if [ ! -f /tmp/get-pip.py ]; then
	curl -sS https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
fi
.venv/bin/python /tmp/get-pip.py --quiet

echo "=== 2) dış bağımlılıkları klonla ==="
mkdir -p external && cd external
[ -d cyclonedds ] || git clone --depth 1 -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds.git
[ -d unitree_mujoco ] || git clone --depth 1 https://github.com/unitreerobotics/unitree_mujoco.git
[ -d unitree_sdk2_python ] || git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git
[ -d unitree-sim2real ] || git clone --depth 1 https://github.com/shivam-sood00/unitree-sim2real.git
[ -d unitree_rl_gym ] || git clone --depth 1 https://github.com/unitreerobotics/unitree_rl_gym.git
cd "$ROOT"

echo "=== 3) Cyclone DDS C kütüphanesini derle (local prefix) ==="
cd external/cyclonedds
mkdir -p build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install -DBUILD_TESTING=OFF
cmake --build . --target install -j"$(nproc)"
cd "$ROOT"

echo "=== 4) Python paketleri ==="
export CYCLONEDDS_HOME="$ROOT/external/cyclonedds/install"
export CMAKE_PREFIX_PATH="$CYCLONEDDS_HOME:$CMAKE_PREFIX_PATH"
.venv/bin/python -m pip install mujoco numpy pyyaml pygame
.venv/bin/python -m pip install cyclonedds==0.10.2
.venv/bin/python -m pip install -e external/unitree_sdk2_python
.venv/bin/python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
.venv/bin/python -m pip install onnxruntime         # ONNX vy policy (RLRunnerOnnx; Gate D/E --onnx)
.venv/bin/python -m pip install "rsl-rl-lib==2.2.4" # for experiments/ vy runner (walk.pt)

echo "=== 5) Go2 sahnelerini kur + config.py yama ==="
GO2DIR="external/unitree_mujoco/unitree_robots/go2"
cp scenes/go2_scene_clean.xml "$GO2DIR/scene_clean.xml"
cp scenes/go2_scene_obstacle.xml "$GO2DIR/scene_obstacle.xml"
cp scenes/go2_rangefinder.xml "$GO2DIR/go2_rangefinder.xml"         # rangefinder sensörlü robot (Gate E --rf)
cp scenes/go2_scene_obstacle_rf.xml "$GO2DIR/scene_obstacle_rf.xml" # engel + rangefinder sahnesi (Gate E --rf)
bash scripts/use_scene.sh clean

echo "=== 6) diasAiMaster Go2 velocity ONNX policy'sini indir (RLRunnerOnnx için) ==="
HF=https://huggingface.co/diasAiMaster/unitree-go2-velocity-flat/resolve/main
MDDIR="external/diasAiMaster_go2_velocity_flat"
mkdir -p "$MDDIR/params"
[ -f "$MDDIR/policy.onnx" ] || curl -sL "$HF/policy.onnx" -o "$MDDIR/policy.onnx"
[ -f "$MDDIR/policy.onnx.data" ] || curl -sL "$HF/policy.onnx.data" -o "$MDDIR/policy.onnx.data"
for f in deploy env agent; do
	[ -f "$MDDIR/params/$f.yaml" ] || curl -sL "$HF/params/$f.yaml" -o "$MDDIR/params/$f.yaml"
done

echo "=== 7) Gate A doğrulama ==="
.venv/bin/python -c "import mujoco, cyclonedds, unitree_sdk2py, torch, onnxruntime; print('Gate A PASS')"

echo "Kurulum tamam. Simülatörü başlat:"
echo "  cd external/unitree_mujoco/simulate_python && DISPLAY=:1 ../../../../.venv/bin/python unitree_mujoco.py"
