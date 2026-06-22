# Deneyler — Go2 düz-çizgi yürüyüşü için vy-tracking politikası araştırması

Bu klasör, Go2'yu daha düz ("normal bir yürüyüş gibi") yürütmek için yapılan
`vy` (yanal hız) destekli politikaların araştırılmasını içerir. Nihai
çözüm olarak **kabul edilmedi**; sonuçlar aşağıda.

## Sonuç özeti

| Politika | vy desteği | unitree_mujoco'da çalışır mı? | Sonuç |
| --- | --- | --- | --- |
| `all_gait_23Dec2025.pt` (shivam, repo varsayılanı) | ✗ (vy ölü) | ✅ stabil | **kabul edildi** — yaw-bias feedforward ile ~0.18 m sürüklenme (5 m'de ~%3-4), hedefe ulaşır |
| `walk.pt` (saifahmadgit, Genesis-eğitim) | ✅ omni | ✗ durur ama ilerlemez | reddedildi — Genesis→unitree_mujoco sim-to-sim boşluğu |
| `amble_with_yaw.pt` (shivam) | ✅ (vy=+0.4 → +0.68 m yanal) | ✅ ayakta ama nav ile dengesiz | reddedildi — kapalı-çevrim nav ile dengesizle (heading loop'u ile düştü; olmadan +5.3 m yanal saptı) |

## Betikler

- `straight_walk_vy.py` — `walk.pt` + `RLRunnerVy` ile düz yürüyüş denemesi
  (walk.pt bu sim'de yürümediği için ileriye gitmedi).
- `straight_walk_amble.py` — `amble_with_yaw` + gerçek `vy` yanal kontrolü
  ile düz yürüyüş denemesi (nav ile dengesizleşti).

`codeep/locomotion/rl_runner_vy.py` geçerli bir vy-runner modülüdür (walk.pt'i
yükler/çalıştırır); ancak walk.pt politikası unitree_mujoco fizik köprüsüne
transfer olmamaktadır (Genesis'te eğitilmiş). Farklı bir unitree_mujoco-
uyumlu vy politikası bulunursa bu runner ile kullanılabilir.

## Sonuç: ONNX vy policy KABUL EDİLDİ (repo ana hattına taşındı)

`experiments/straight_walk_onnx.py` probe'u, `diasAiMaster/unitree-go2-velocity-flat`
ONNX policy'sinin unitree_mujoco'ya **Python onnxruntime yoluyla** (C++
`go2_ctrl` kurulumu GEREKMEKSİZİN) transfer olduğunu doğruladı. Kabul edildi
ve ana hatta alındı:

- `codeep/locomotion/rl_runner_onnx.py` — `RLRunnerOnnx` (`RLRunner` ile aynı
  `set_command(vx,vy,wz)` API'si).
- `gates/straight_walk.py --onnx` — kapalı-çevrim vy ile düz yürüyüş, `yaw_bias`
  yok, 5 m'de ~0.03 m yanal sapma (all_gait+yaw_bias ~0.18 m).
- `gates/gate_e_obstacle.py --onnx --rf` — sensör-tabanlı engel tespiti + ONNX.

Aşağıdaki "daha ileri için" notu artık geçmişe dönüktür — C++ ağır kurulum
gerekmedi, Python ONNX yeterli oldu.

## Daha ileri için (README §10)

unitree_mujoco DDS köprüsü için tasarlanmış, `vy` destekli gerçek bir düz
yürüyüş için: `unitree_rl_mjlab` Go2 velocity politikası (diasAiMaster HF) +
C++ `go2_ctrl` deploy'u. Bu, ağır bir kurulum (cmake + mjlab + ONNX) gerektirir
ama köprü için tasarlandığından gerçekten düz yürür. Mevcut iterasyonun
kapsamı dışında bırakıldı.
