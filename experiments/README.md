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

## Daha ileri için (README §10)

unitree_mujoco DDS köprüsü için tasarlanmış, `vy` destekli gerçek bir düz
yürüyüş için: `unitree_rl_mjlab` Go2 velocity politikası (diasAiMaster HF) +
C++ `go2_ctrl` deploy'u. Bu, ağır bir kurulum (cmake + mjlab + ONNX) gerektirir
ama köprü için tasarlandığından gerçekten düz yürür. Mevcut iterasyonun
kapsamı dışında bırakıldı.
