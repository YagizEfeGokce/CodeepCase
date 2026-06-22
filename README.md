# Codeep — Unitree Go2 Simülasyon Case

Teknik rapor · Unitree Go2 robotunu MuJoCo simülasyonunda çalıştırma, hedefe
yönlendirme ve engel algılama

> Bu rapor, Codeep STAJ programı teknik case'inin teslim dokümanıdır. Case
> brief'i: `Codeep_Teknik_Case.pdf`. Tüm zorunlu çıktılar ve bonus maddeleri
> (engel algılama, çoklu waypoint, modüler kod, G1 humanoid denemesi)
> yerine getirilmiştir.
>
> Repo: <https://github.com/YagizEfeGokce/CodeepCase>

## Hızlı başlangıç

```bash
git clone https://github.com/YagizEfeGokce/CodeepCase.git && cd CodeepCase
bash scripts/setup.sh          # tek komutla kurulum (sudo'suz, ~3 dk)
bash scripts/check_env.sh       # ortamı doğrula (Go2 + G1)

bash run.sh b                  # Gate B: Go2 dik dur
bash run.sh c --vx 0.3          # Gate C: ileri yürü (all_gait trot)
bash run.sh d --onnx            # Gate D: düz yürüyüş — ONNX vy policy (çekirdek; yaw_bias yok, ~0.03 m sapma)
bash run.sh d                   # Gate D: (yedek) all_gait+yaw_bias ~0.18 m sapma
bash run.sh e --onnx --rf        # Gate E: sensör-tabanlı (rangefinder) + ONNX vy policy (bonus)
bash run.sh e                   # Gate E: engelden kaçın (harita-tabanlı, all_gait)
bash run.sh f                   # Gate F+: 4 waypoint + engel (tek run)
bash run.sh course --onnx --rf  # Gate Course: 5 waypoint + 3 engel (sensör detour, bonus)
bash run.sh g1                  # Gate G: G1 humanoid yürü (bonus)
```

**Düz-çizgi yürüyüş Go2'muzün çekirdek yeteneğidir** ve ONNX vy-tracking
policy'si (`run.sh d --onnx`) ile sağlanır: kapalı-çevrim `vy` yanal düzeltme,
`yaw_bias` önyargısı yok, 5 m'de ~0.03 m sapma. `run.sh <gate>` simülatörü doğru
sahneyle başlatır, gate'i çalıştırır ve kapatır — ikinci bir terminal gerekmez.
Daha ince kontrol için §4'teki tek-tek komutlar kullanılabilir.

### Docker ("benim makinemde çalışıyor" sendromu için)

Tüm ortamı içine gömülü bir imaj — her cihazda aynı şekilde çalışır (X/GPU
 gerekmez):

```bash
docker build -t codeep .
docker run --rm codeep                 # Gate B headless (smoke test)
docker run --rm codeep bash run.sh f   # 4 waypoint + engel (headless)
```

Ayrıntı (GUI viewer dahil): `docs/docker.md`.

---

## 1. Kullanılan simülasyon ortamı

| Bileşen | Seçim | Sürüm | Kaynak |
| --- | --- | --- | --- |
| Fizik motoru | **MuJoCo** | 3.9.0 | `pip install mujoco` |
| Simülatör | **unitree_mujoco** (`simulate_python`) | upstream main | `unitreerobotics/unitree_mujoco` |
| DDS katmanı | **Cyclone DDS** | 0.10.5 (C kütüphanesi kaynaktan derlendi) | `eclipse-cyclonedds/cyclonedds` |
| Robot SDK | **unitree_sdk2_python** | 1.0.1 | `unitreerobotics/unitree_sdk2_python` |
| Ara katman (DDS IDL) | unitree_go (Go2) / unitree_hg (G1) | — | SDK ile gelir |
| OS | Ubuntu 24.04 (Noble) | — | konak makine |
| Python | 3.12 (izole `.venv`) | — | `python3 -m venv` |

Simülatör, MuJoCo viewer'ı pasif modda açar ve bir DDS köprüsü
(`unitree_sdk2py_bridge.py`) çalıştırır: `rt/lowcmd`'ye abone olur,
`rt/lowstate` + `rt/sportmodestate` + `rt/wirelesscontroller` konularını
yayımlar. `SportModeState.position/velocity` doğrudan robotun dünya
koordinatlarındaki konumunu verir; bu sayede navigasyon katmanı ek bir SLAM
ya da durum-kestirimi gerektirmeden pozu okur.

**Neden MuJoCo + unitree_mujoco?** Resmî Unitree simülatörüdür ve gerçek
robotla aynı düşük-seviye arayüzü (`LowCmd`/`LowState`) kullanır; böylece
sim-to-real geçişi doğrudandır. Ayrıntılı gerekçe §9'da.

## 2. Kullanılan robot modeli

- **Ana robot: Unitree Go2** — 12 serbestlik dereceli (4 bacak × 3 eklem:
  FR, FL, RR, RL). MJCF: `unitree_mujoco/unitree_robots/go2/go2.xml`.
  Bacak geometrisi: uyluk 0.213 m, baldır 0.213 m, kalça yanal ofset 0.0955 m.
- **Bonus robot: Unitree G1** — 29 DOF humanoid. MJCF:
  `unitree_rl_gym/resources/robots/g1_description/`. G1, `unitree_hg` IDL'sini
  kullanır; Gate G'de ayrı bir pipeline ile çalıştırıldı (§6.2).

## 3. Kurulum adımları

Tüm bağımlılıklar `external/` altına klonlanır; ana repo sade kalır.

```bash
git clone https://github.com/YagizEfeGokce/CodeepCase.git CodeepCase && cd CodeepCase

# A) En kolay yol: tek komutluk kurulum (venv + 5 dış bağımlılık + CycloneDDS derle + pip kur)
bash scripts/setup.sh
# B) Doğrula:
bash scripts/check_env.sh
```

`scripts/setup.sh` tam olarak şunları yapar (elle kurulum istenirse):

```bash
# 1) Sanal ortam (ensurepip olmayan Debian/Ubuntu için --without-pip yolu)
python3 -m venv --without-pip .venv
curl -sS https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
.venv/bin/python /tmp/get-pip.py

# 2) Dış bağımlılıkları klonla
mkdir -p external && cd external
git clone --depth 1 -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds.git
git clone --depth 1 https://github.com/unitreerobotics/unitree_mujoco.git
git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git
git clone --depth 1 https://github.com/shivam-sood00/unitree-sim2real.git
git clone --depth 1 https://github.com/unitreerobotics/unitree_rl_gym.git
cd ..

# 3) Cyclone DDS C kütüphanesini derle (sudo gerekmez, local prefix)
cd external/cyclonedds && mkdir -p build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install -DBUILD_TESTING=OFF
cmake --build . --target install -j$(nproc)
cd ../../..

# 4) Python paketlerini venv'e kur
export CYCLONEDDS_HOME="$PWD/external/cyclonedds/install"
export CMAKE_PREFIX_PATH="$CYCLONEDDS_HOME:$CMAKE_PREFIX_PATH"
.venv/bin/python -m pip install mujoco numpy pyyaml pygame
.venv/bin/python -m pip install cyclonedds==0.10.2
.venv/bin/python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
.venv/bin/python -m pip install onnxruntime  # ONNX vy policy (RLRunnerOnnx; Gate D/E --onnx)
.venv/bin/python -m pip install -e external/unitree_sdk2_python  # (editable)

# 5) Go2 sahnelerini kur (clean/obstacle + rangefinder'lı rf sahnesi) + config.py yama
GO2DIR="external/unitree_mujoco/unitree_robots/go2"
cp scenes/go2_scene_clean.xml      "$GO2DIR/scene_clean.xml"
cp scenes/go2_scene_obstacle.xml   "$GO2DIR/scene_obstacle.xml"
cp scenes/go2_rangefinder.xml     "$GO2DIR/go2_rangefinder.xml"        # rangefinder sensörlü robot
cp scenes/go2_scene_obstacle_rf.xml "$GO2DIR/scene_obstacle_rf.xml"  # engel + rangefinder sahnesi
bash scripts/use_scene.sh clean

# 6) diasAiMaster Go2 velocity ONNX policy'sini indir (RLRunnerOnnx için)
HF=https://huggingface.co/diasAiMaster/unitree-go2-velocity-flat/resolve/main
MDDIR="external/diasAiMaster_go2_velocity_flat"; mkdir -p "$MDDIR/params"
curl -sL "$HF/policy.onnx"        -o "$MDDIR/policy.onnx"
curl -sL "$HF/policy.onnx.data"    -o "$MDDIR/policy.onnx.data"
for f in deploy env agent; do curl -sL "$HF/params/$f.yaml" -o "$MDDIR/params/$f.yaml"; done
```

Kurulumu doğrula (Gate A):

```bash
.venv/bin/python -c "import mujoco, cyclonedds, unitree_sdk2py, torch; print('OK')"
```

> Not: Konak makinede `pip` ve `ensurepip` eksikti (PEP 668). Yukarıdaki
> `--without-pip` + `get-pip.py` yolu sudo gerektirmeden izole bir venv kurar.

Otomatik kurulum için: `bash scripts/setup.sh`.

## 4. Çalıştırma adımları

Her gate ayrı bir doğrulama betiğidir. Simülatör ayrı bir terminalde çalışır;

```bash
# --- A) Simülatörü başlat (Go2, engelsiz sahne) ---
cd external/unitree_mujoco/simulate_python
DISPLAY=:1 ../../../../.venv/bin/python unitree_mujoco.py
```

`simulate_python/config.py` içinde `ROBOT` ve `ROBOT_SCENE` ayarlanır. Sahne
`scripts/use_scene.sh` ile değiştirilir:

```bash
bash scripts/use_scene.sh clean      # scene_clean.xml — Gate B/C/D (engelsiz)
bash scripts/use_scene.sh obstacle   # scene_obstacle.xml — Gate E/F+ (kutu engel)
bash scripts/use_scene.sh rf         # scene_obstacle_rf.xml — Gate E --rf (engel + rangefinder sensörleri)
bash scripts/use_scene.sh course     # scene_course.xml — Gate Course --rf (5 waypoint + 3 engel)
```

- `scene_clean.xml` — engelsiz sahne (Gate B/C/D ve çoklu waypoint testleri)
- `scene_obstacle.xml` — yolda tek kutu engel (Gate E/F+, harita-tabanlı)
- `scene_obstacle_rf.xml` — kutu engel + Go2'ye 3 MuJoCo `<rangefinder>` sensörü
  (ön/sol/sağ); `sim_headless.py` bu ölçümleri DDS `rt/rangefinders` konusunda
  yayımlar (Gate E `--rf` sensör-tabanlı tespit)
- `scene_course.xml` — 5-waypoint kurs + 3 kutu engel (her biri bir bacak üzerinde,
  0.6 m boy); Gate Course `--rf` sensör-tabanlı çoklu-engel detour

```bash
# --- B) Doğrulama betikleri (gates/, ayrı terminal, aynı venv) ---
.venv/bin/python gates/gate_b_stand.py        # Gate B: dik dur
.venv/bin/python gates/gate_c_rl.py --vx 0.3 # Gate C: ileri yürü (RL trot)
.venv/bin/python gates/straight_walk.py --onnx   # Gate D: düz yürüyüş (çekirdek) — ONNX vy policy, yaw_bias yok
.venv/bin/python gates/straight_walk.py      # Gate D: (yedek) hedefe git (5,0) — all_gait+yaw_bias
.venv/bin/python gates/gate_e_obstacle.py --onnx --rf   # Gate E: sensör-tabanlı (rangefinder) + ONNX vy policy (bonus)
.venv/bin/python gates/gate_e_obstacle.py    # Gate E: engelden kaçın (harita-tabanlı)
.venv/bin/python gates/gate_f_combined.py   # Gate F+: 4 waypoint + engel (tek run)
.venv/bin/python gates/gate_course.py --onnx --rf  # Gate Course: 5 waypoint + 3 engel (sensör detour, bonus)
.venv/bin/python gates/gate_g_g1.py          # Gate G: G1 humanoid (kendi viewer'ı)

# İzleme/demo yardımcıları (scripts/):
.venv/bin/python scripts/stand_watch.py     # Go2 dik durma canlı izle
.venv/bin/python scripts/walk_watch.py      # Go2 ileri yürüyüşü canlı izle
```

G1 (Gate G) kendi MuJoCo viewer'ını açar; Go2 simülatörüne ihtiyaç duymaz.

## 5. Robot kontrol yaklaşımı

Katmanlı mimari (her katman `codeep/` altında ayrı modül):

```
┌─────────────────────────────────────────────────────────────┐
│  codeep/control/waypoints.py           — WaypointManager (Gate F)   │
│  codeep/control/avoider.py             — ObstacleAvoider (Gate E, harita) │
│  codeep/control/rangefinder_avoider.py — RangefinderAvoider (Gate E --rf, sensör) │
│  codeep/control/nav.py                 — NavController (Gate D)      │
├─────────────────────────────────────────────────────────────┤
│  codeep/locomotion/rl_runner_onnx.py — RLRunnerOnnx (ONNX vy policy — ÇEKİRDEK düz yürüyüş) │
│  codeep/locomotion/rl_runner.py      — RLRunner (all_gait trot; Gate C/F, vy ölü)  │
│    pre-trained Go2 gait policy  →  set_command(vx,vy,wz)        │
├─────────────────────────────────────────────────────────────┤
│  codeep/robot/go2_client.py      — DDS köprüsü (LowCmd/LowState)    │
│  codeep/robot/rangefinder_idl.py — RangefinderData DDS IDL (rt/rangefinders) │
├─────────────────────────────────────────────────────────────┤
│  unitree_mujoco / scripts/sim_headless.py — MuJoCo + DDS bridge     │
└─────────────────────────────────────────────────────────────┘
```

- **Locomotion (Gate C/D/E):** Go2'nin yürüme hareketi önceden eğitilmiş
  RL politikalarıyla üretilir; her ikisi de aynı `set_command(vx, vy, wz)`
  API'sini sunar (gerçek Go2 sport-mode paradigması: yürüyüş kara kutu,
  dışarısı hız komutu gönderir):
  - `RLRunnerOnnx` (`--onnx`, **çekirdek düz yürüyüş policy'si**) —
    `diasAiMaster/unitree-go2-velocity-flat` ONNX policy'si (mjlab PPO, düz
    zemin). `vy`'yi **izler**; normalize katmanı ONNX grafiğine katlanmış
  olduğundan ham 45-boyutlu obs beslenir. `vy` desteği sayesinde
    `NavController` gerçek kapalı-çevrim yanal düzeltme yapar → `yaw_bias`'a
    ihtiyaç duymadan düz yürüyüş (Gate D `--onnx`: 5 m'de ~0.03 m sapma).
    **Bu, Go2'müzün çekirdek yürüyüş yeteneğidir** (düz-çizgi, kapalı-çevrim
    yanal düzeltme, yaw_bias yok). PDF'in "RL beklenmiyor" notu "zorunlu değil"
    anlamındadır; navigasyon/engel/waypoint/dokümantasyon bize aittir (§9).
  - `RLRunner` (yedek/eski, Gate C + F) — `all_gait_23Dec2025.pt` trot
    policy'si (shivam-sood00/unitree-sim2real). `vy` komutu **ölüdür**; yanal
    crab `yaw_bias` ön-beslemesiyle iptal edilir (5 m'de ~0.18 m sapma, §9.5).
    Gate C (ileri yürü) ve Gate F+ (waypoint, all_gait) bunu kullanır;
    düz-çizgi yürüyüş için `RLRunnerOnnx` tercih edilir.
- **Navigation (Gate D, bizim kodumuz):** `NavController`, P-kontrol
  tabanlı dümenleme yapar: hedefe bearing ile yaw hatası → `wz`; mesafe →
  `vx` (hizalanma olmadığında `min_align` kadar minimum ileri hız, böylece
  policy yürümeyi bırakmaz). Yanal crab iptali: çekirdek yürüyüş policy'si
  (ONNX, `use_vy=True`) gerçek `vy` ile kapalı-çevrim düzeltir (`yaw_bias` yok);
  yedek `all_gait` (`use_vy=False`, `vy` ölü) `yaw_bias` ön-beslemesiyle iptal
  eder (§9.5). Poz `SportModeState`'ten okunur.
- **ObstacleAvoider (Gate E, bizim):** iki mod — (i) **harita-tabanlı**
  (`avoider.py`, varsayılan): engellerin konumu sahne/config'den bilinir,
  gövde çerçevesine projekte edilip ön+reaksiyon mesafesi içinde ise tespit
  edilir, engelin yanına detour waypoint'ine yönlendirilir, ardından hedefe
  döner. (ii) **sensör-tabanlı** (`rangefinder_avoider.py`, `--rf`): engel
  konumu bilinmez; Go2'nin 3 MuJoCo rangefinder'ı önündeki mesafeyi ölçer,
  `forward < reaction_dist` iken tespit edilir, sol/sağ rangefinder'ı daha
  net olan tarafa detour yapılır (§7). PDF'in "dur veya yön değiştir" şartı
  "yön değiştir" ile sağlanır.
- **WaypointManager (Gate F, bizim):** waypoint'lere sırayla gider; varışı
  poz mesafesinden ölçer ve `NavController` `(0,0,0)` göndermeden bir
  sonrakine geçer (policy'nin donmasını önler, §9.5).
- **G1 (Gate G, bonus):** `unitree_rl_gym`'in `deploy_mujoco` pipeline'ı;
  önceden eğitilmiş `g1/motion.pt` hareket politikası, kendi MJCF'i ve
  viewer'ı ile standalone çalışır.

Tüm modüller `codeep/` paketinde; doğrulama betikleri `gates/` altında.

## 6. Hedef noktaya yönlendirme mantığı

`NavController.step()` her döngüde (≈10 Hz):

1. `pose = SportModeState.position`, `yaw = IMU quaternion → yaw`.
2. Hedefe vektör: `dx = tx - px`, `dy = ty - py`, `dist = hypot(dx,dy)`.
3. `des_heading = atan2(dy, dx) + yaw_bias` (yaw_bias: policy'nin sağa
   crab'ini ön-besleme ile iptal eden sabit ofset, §9.5).
4. `yaw_err = wrap(des_heading - yaw)`; `wz = clip(kp_yaw*yaw_err +
   kp_lat_yaw*lat_body, ±max_wz)`.
5. `align = max(min_align, 1 - |yaw_err|/slow_yaw_err)`; `vx = clip(kp_lin*dist,
   max_vx) * align` — büyük yaw hatasında bile minimum ileri hız korunur.
6. `vy = clip(kp_lat*lat_body, ±max_vy)` (çekirdek ONNX vy policy,
   `use_vy=True`) — yanal sapmayı kapalı-çevrim düzeltir; **veya** `vy = 0`
   (yedek all_gait, `use_vy=False`, `vy` ölü).
7. `dist <= goal_tol` → hedefe ulaşıldı.

Bu denetleyici, Gate D'de (5,0) hedefine ulaştı: çekirdek ONNX `--onnx` —
varış 0.24 m, **max yanal sapma 0.03 m**, `yaw_bias` yok; yedek all_gait+yaw_bias
— varış 0.24 m, ~0.18 m yanal sapma. Gate F+'da 4 waypoint'i sırayla gezdi
(maks varış hatası 0.30 m).

## 7. Engel algılama yaklaşımı (varsa)

İki modu var; her ikisi de aynı reaksiyon/planning kalıbını (durma yok,
engelin yanına detour waypoint'i → ardından hedefe dön) kullanır — sadece
**tespit kaynağı** değişir.

### 7.1 Sensör-tabanlı (bonus yol, `--rf`)

`RangefinderAvoider` (Gate E `--onnx --rf`):

- **Sensör:** `go2_rangefinder.xml`, Go2 `base_link`'ine 3 MuJoCo
  `<rangefinder>` sensörü ekler: `rf_center` (ön, +x), `rf_left`/`rf_right`
  (ön-sol / ön-sağ, ±30°). `scripts/sim_headless.py` her adımda bu
  ölçümleri okuyup DDS `rt/rangefinders` konusunda `RangefinderData(forward,
  left, right)` olarak yayımlar (hit yoksa 99.0 m). Rangefinder ışını
  site'nin +z ekseni boyunca ilerler; robotun kendi gövdesi hariç tutulur.
- **Tespit:** `RangefinderAvoider` `rt/rangefinders`'a abone olur. Her
  döngüde `forward < reaction_dist` (1.0 m) iken engel tespit edilir —
  engel konumu **bilinmez**, sensörden gelir. Sol/sağ mesafesine bakılıp
  tarafsız (daha net) taraf seçilir (`side = +1` sol, `-1` sağ).
- **Reaksiyon (yön değiştir):** tespit anında, heading'e dik `detour_dist`
  (0.8 m) uzaklıkta, seçilen tarafa bir detour waypoint'i hesaplanır
  (`detour = pose + R(±90°)·detour_dist`). Robot `NavController` ile oraya
  gider. **Waypoint'e ulaşınca** (erken değil — burası önemli: forward ışını
  temizlenince değil) `to_final` durumuna geçer ve asıl hedefe döner. Yolda
  yeni bir engel görülürse (`forward < reaction_dist`) tekrar detour yapar.
  `stop_time = 0` (PDF "dur **veya** yön değiştir" — biz yön değiştiririz).
- **Sonuç (Gate E `--onnx --rf`):** (2.5, 0)'daki kutu, robot ~1.2 m'deyken
  `forward=1.00 m` ile tespit edildi; robot sola detour yaptı (1.2, 0.8),
  kutuyu 0.42 m clearance ile geçip (5, 0) hedefine ulaştı (varış 0.24 m,
  çarpmadı, düşmedi).

### 7.2 Harita-tabanlı (`avoider.py`, varsayılan)

`ObstacleAvoider` (Gate E `--onnx`'siz / F+):

- **Tespit:** engeller `(x, y, r)` listesi config'den bilinir. Her adımda
  her engel gövde çerçevesine projekte edilir (`bx = cos·dx + sin·dy` ileri,
  `by = -sin·dx + cos·dy` yanal). `bx > 0` ve `bx < reaction_dist` (1.0 m) ve
  `|by| < r + clearance` ise engel tespit edilir. Poz `SportModeState`'ten.
- **Reaksiyon:** tespit anında hedef, engelin yanındaki detour waypoint'ine
  (`ox, oy + (r + margin)`) geçer; robot oraya gider, ardından asıl hedefe
  döner.
- **Sonuç (Gate E):** (2.5, 0)'daki kutu, robot (1.5, 0) civarında tespit
  edildi, robot (2.5, 0.7) detour'una sapıp kutuyu 0.39 m clearance ile
  geçti ve (5, 0) hedefine ulaştı (varış 0.24 m).

> Sensör-tabanlı mod, §10'daki "gerçek sensör ile engel algılama" maddesini
> gerçekleştirir; reaksiyon/planning katmanı harita-tabanlıyla aynı kalır,
> sadece tespit kaynağı değişmiştir.

## 8. Karşılaşılan problemler ve çözüm yaklaşımı

1. **Gazebo/ROS2 denemesinde Go2 ayağa kalkmıyordu.** Robot, düz bacaklarla
   spawn olup düşüyordu (önceki `Codeep/` reposundaki CHAMP + ros2_control
   yaklaşımı). *Çözüm:* resmî Unitree simülatörü unitree_mujoco'ya geçildi;
   burada `LowCmd` ile bükük bacak duruş pozu doğrudan komutlanır ve Go2
   dik durur (Gate B: 30 sn, 7 mm sürüklenme).
2. **Open-loop IK trot, unitree_mujoco'nun tork-PD köprüsünde yürümedi.**
   8 deneme: ayak zeminde kaydığından gövde istenen yöne değil planlanan
   yönün tersine sürükleniyordu; duruş düzgün ama ileri hareket kontrol
   edilemiyordu. *Çözüm:* locomotion katmanı önceden eğitilmiş RL trot
   politikasına bırakıldı; navigasyon bizim kodumuzla (§9.2).
3. **pip / ensurepip eksik + PEP 668.** *Çözüm:* `--without-pip` venv +
   `get-pip.py` ile sudo'suz izole kurulum; CycloneDDS C kütüphanesi local
   prefix'e derlendi.
4. **all_gait policy'sinde `vy` komutu ölü.** Saf `vy=0.3` sadece 3.4 cm
   yanal ama 19 cm ileri hareket verdi. *Çözüm:* yanal crab iptali `vy`
   yerine ön-besleme `yaw_bias` ile yapıldı.
5. **Policy `(0,0,0)` komutunda donuyor.** Hem "stop" hem de waypoint
   varışında sıfır komut policy'yi durduruyordu. *Çözüm:* dur-komutu
   kaldırıldı (stop_time=0); `WaypointManager` varışı poz mesafesinden
   ölçüp `NavController`'ın kendi goal_tol'una varmadan bir sonraki
   waypoint'e geçer (nav.goal_tol=0.15 < wm.goal_tol=0.30).
6. **İleri yürürken sağa crab (~0.025 m/s).** *Çözüm:* heading setpoint'ine
   `yaw_bias = 0.16 rad` ön-besleme eklendi; yanal sürüklenme 0.30 m → 0.17 m
   (5 m'de). Tam sıfırlanmadı (§10).
7. **90° dönüşlerde robot yere basıp duruyordu.** *Çözüm:* `min_align = 0.35`
   ile hizalanma olmadığında bile minimum ileri hız korundu; policy yürüyerek
   döner (yerinde dönemiyor).
8. **cyclonedds `RangefinderData` IDL'si populate edilemedi** (`Type float as
   used in codeep.robot.rangefinder_idl cannot be resolved`). *Sebep:*
   `from __future__ import annotations` ek olarak hint'leri string yapıyordu;
   cyclonedds `cls.__annotations__`'ı ham okuduğundan `"float"`'u modülden
   import etmeye çalışıp başarısız oluyordu. *Çözüm:* `rangefinder_idl.py`'den
   `__future__` importu kaldırıldı (hint'ler gerçek builtin tip
   `float`/`str` oldu). IdlStruct modüllerine bir daha `from __future__ import
   annotations` eklenmemeli.
9. **Rangefinder sürekli 99 (hit yok) okuyordu — engel algılanmıyordu.**
   *Sebep:* rf site world z ≈ 0.47 m (gövde 0.42 + 0.05), kutu sadece 0.15 m
   boyundaydı; yatay ışın kutunun **üzerinden** geçiyordu. *Çözüm:* engel
   yüksek yapıldı (`size="0.15 0.15 0.3"`, 0.6 m) — göğüs yüksekliğindeki
   ön rangefinder'ı artık görüyor.
10. **`RangefinderAvoider` engel tespit edip detour'a geçti ama hemen
    `to_final`'a dönüp kutuya çarparak durdu.** *Sebep:* `to_detour → to_final`
    geçişi `forward >= clear_dist` olunca yapıyordu; robot burnu yana dönünce
    forward ışını temizleniyor, henüz detour waypoint'ine ulaşmadan hedefe
    dönüyordu. *Çözüm:* geçiş sadece `_wp_reached` olunca yapıldı; `to_final`'da
    yeniden tespit (forward < reaction_dist → tekrar detour) eklendi.
11. **`sim_headless.py --viewer`'da `launch_passive` "core dumped" verdi.**
    *Sebep:* çökme sadece **kapanışta** (`viewer.close()` / interpreter exit)
    oluyordu, çalışma sırasında değil. *Çözüm:* cosmetic — simülasyon + viewer +
    rangefinder yayımı çalışma süresince normal; gate bittikten sonraki
    kapanış çökmesi sonucu etkilemiyor. `import mujoco.viewer` modül
    top-level'e taşındı (lazy import launch_passive'ı bozuyordu).

## 9. Kullanılan kaynaklar

- `unitreerobotics/unitree_mujoco` — resmî MuJoCo simülatörü + Go2/G1 MJCF.
- `unitreerobotics/unitree_sdk2_python` — Go2 düşük-seviye DDS arayüzü.
- `eclipse-cyclonedds/cyclonedds` — DDS ara katmanı (0.10.x).
- `shivam-sood00/unitree-sim2real` — Go2 önceden eğitilmiş trot
  politikaları + sim2sim runner (RLRunner bunu sarmalar).
- `diasAiMaster/unitree-go2-velocity-flat` (Hugging Face) — Go2 mjlab PPO
  velocity policy'si (ONNX; `RLRunnerOnnx` bunu yükler, Gate D/E `--onnx`).
  Normalize katmanı ONNX grafiğine katlanmış; 45-boyutlu obs, 12 eylem,
  `action_scale=0.5`, `vy` desteği.
- `onnxruntime` — ONNX policy çıkarımı (CPU yeterli).
- `unitreerobotics/unitree_rl_gym` — G1 önceden eğitilmiş hareket
  politikası + `deploy_mujoco` pipeline (Gate G).
- `darshmenon/quadruped-dog-rl` — open-loop IK gait referansı (bilgi
  kaynağı; bizim open-loop denememizi şekillendirdi, nihayet kullanılmadı).
- MuJoCo docs (mujoco.readthedocs.io), Unitree developer docs
  (support.unitree.com/home/en/developer).

## 10. Geliştirilebilecek noktalar

- ~~**Gerçek sensör ile engel algılama:**~~ **TAMAMLANDI** — §7.1. Go2'ye 3
  MuJoCo `<rangefinder>` sensörü eklendi (`go2_rangefinder.xml`);
  `sim_headless.py` ölçümleri DDS `rt/rangefinders`'ta yayımlar;
  `RangefinderAvoider` (`--rf`) sensörden tespit edip detour yapar (Gate E
  `--onnx --rf` PASS). **Çoklu-engel çoklu-waypoint kursu** da TAMAMLANDI:
  Gate Course `--onnx --rf` — 5 waypoint + 3 engel, her bacakta sensör tespit
  - reaktif detour (PASS: 5/5 wp, 4 detour, min_obs 0.33 m). Kalan: daha
  geniş açılı tarama / 360° lidar ve çoklu-engel A* global planlayıcı (aşağıda).
- ~~**`vy` destekleyen policy:**~~ **TAMAMLANDI** — `RLRunnerOnnx`
  (`diasAiMaster` ONNX velocity policy) `vy`'yi izler; `NavController`
  `use_vy=True` ile kapalı-çevrim yanal düzeltme yapar → `yaw_bias`
  gerekmez (Gate D `--onnx`: 5 m'de ~0.03 m sapma). (Önceki denemeler:
  `experiments/` — `walk.pt` Genesis→unitree_mujoco transfer olmamış,
  `amble_with_yaw` nav ile dengesizdi.)
- **G1'i DDS köprüsüne entegre etmek:** G1'i kendi standalone pipeline'ından
  `unitree_mujoco` DDS köprüsüne taşımak (aynı navigasyon stack'ini
  paylaşır).
- **A\* global planlayıcı:** sensör-tabanlı mod şu an tek engel ve
  Gate Course'taki 3 engel için **reaktif** detour yapıyor (her bacakta
  `RangefinderAvoider` tespit → yan detour → hedefe dön); sık/karmaşık
  engel alanları (labirent) için A* (önceki Gazebo denemesinde mevcuttu)
  portlanıp `RangefinderAvoider`'a bağlanabilir.
- **Daha geniş rangefinder tarama:** 3 ışın yerine dönen lidar / derinlik
  sensörü ekleyip tespit açısını büyütmek.
- ~~**ROS2 topic/service katmanı:**~~ **TAMAMLANDI (bonus)** — §11. `codeep/ros2_bridge.py`
  DDS<->ROS2 köprü düğümü: Go2 DDS arayüzünü standard ROS2 topic/service'lara
  açar (`/go2/cmd_vel`, `/go2/pose`, `/go2/imu`, `/go2/joint_states`,
  `/go2/range_*`, `/go2/stop`). `RLRunnerOnnx(ros2_cmd=True)` `/go2/cmd_vel` ile
  sürülür. `bash scripts/ros2_demo.sh` ile uçtan uca doğrulandı.
- **Demo videosu:** kullanıcı tarafından hazırlanacak.

## 11. ROS2 topic/service köprüsü (bonus)

Mevcut DDS arayüzünü (`unitree_sdk2py` + CycloneDDS) standard ROS2 topic/service'lara
açan bir köprü düğümü — case'in "ROS2 topic/service yapısının etkin kullanılması"
bonus maddesi. ROS2 Jazzy + `rclpy` üzerinde, `codeep/ros2_bridge.py`:

- **DDS → ROS2 (durum telemetrisi):**
  - `rt/lowstate` → `/go2/joint_states` (`sensor_msgs/JointState`, 12 eklem) + `/go2/imu` (`sensor_msgs/Imu`)
  - `rt/sportmodestate` → `/go2/pose` (`geometry_msgs/PoseStamped`)
  - `rt/rangefinders` → `/go2/range_{forward,left,right}` (`sensor_msgs/Range`)
- **ROS2 → DDS (komut):**
  - `/go2/cmd_vel` (`geometry_msgs/Twist`: `linear.x`=vx, `linear.y`=vy, `angular.z`=wz) → DDS `rt/cmd_vel` (`CmdVel` IDL)
  - `/go2/stop` servisi (`std_srvs/Trigger`) → `rt/cmd_vel` (0,0,0)

`RLRunnerOnnx(ros2_cmd=True)` DDS `rt/cmd_vel`'i abone olup policy'yi ROS2
`cmd_vel` ile sürer; böylece Go2 standart ROS2 `cmd_vel` topic'inden
komutlanır. Köprü system `python3` + ROS2 ile, runner venv `python` ile çalışır
(`scripts/ros2_env.sh` ortamı ayarlar).

```bash
bash scripts/ros2_demo.sh          # sim + runner + köprü + demo istemci (uçtan uca)
# veya tek tek:
source scripts/ros2_env.sh
.venv/bin/python scripts/ros2_run.py            # ONNX runner, ros2_cmd modu
python3 codeep/ros2_bridge.py                   # DDS<->ROS2 köprü düğümü
ros2 topic pub /go2/cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}"   # sür
ros2 service call /go2/stop std_srvs/Trigger                            # dur
ros2 topic echo /go2/pose                                                # telemetri
```

**Doğrulama:** demo istemci `/go2/cmd_vel` ile 6 sn ileri sürdü (x 0.01→1.78 m,
~0.3 m/s), `/go2/stop` servisi robotu durdurdu (pose sabit 1.78 m), `/go2/pose`
telemetrisi okundu — ROS2 topic + service yolu uçtan uca çalıştı.

---

## Ek: Doğrulama (gates) özeti

| Gate | Hedef | Sonuç |
| --- | --- | --- |
| A | Kurulum + import | PASS |
| B | Go2 dik dur (30 sn, düşmez) | PASS (7 mm sürüklenme) |
| C | İleri yürü | PASS (+1.13 m ileri, dik) |
| D `--onnx` | **Çekirdek** düz yürüyüş (5,0) — ONNX vy policy (yaw_bias yok) | PASS (varış 0.24 m, **max yanal 0.03 m**) |
| D | Hedefe ulaş (5,0) — all_gait + yaw_bias (yedek) | PASS (varış 0.24 m, ~0.18 m yanal) |
| E | Engelden kaçın + hedefe ulaş (harita-tabanlı) | PASS (0.39 m clearance) |
| E `--onnx --rf` | Engelden kaçın — **sensör** (rangefinder) tespiti | PASS (0.42 m clearance, 0.24 m varış, düşmedi) |
| F+ | 4 waypoint + engel (tek run) | PASS (4/4, 0.30 m, engel kaçınıldı) |
| Course `--onnx --rf` | 5 waypoint + 3 engel — **sensör** çoklu-engel detour (bonus) | PASS (5/5 wp, 4 sensör detour, min_obs 0.33 m, düşmedi) |
| G | G1 humanoid yürü (bonus) | PASS (+1.22 m, dik) |

## Ek: Repo yapısı

```
CodeepCase/
├── README.md                  # bu teknik rapor (TR)
├── Codeep_Teknik_Case.pdf
├── requirements.txt
├── pyrightconfig.json
├── codeep/                    # kütüphane
│   ├── robot/go2_client.py       # DDS köprüsü (LowCmd/LowState/SportModeState)
│   ├── robot/rangefinder_idl.py  # RangefinderData DDS IDL (rt/rangefinders)
│   ├── robot/cmd_vel_idl.py      # CmdVel DDS IDL (rt/cmd_vel, ROS2 bridge)
│   ├── ros2_bridge.py            # DDS<->ROS2 bridge node (§11, bonus)
│   ├── locomotion/rl_runner_onnx.py # RLRunnerOnnx — ONNX vy policy (ÇEKİRDEK düz yürüyüş, --onnx)
│   ├── locomotion/rl_runner.py      # RLRunner — all_gait trot policy (yedek; Gate C/F, vy ölü)
│   └── control/
│       ├── kinematics.py          # (open-loop deneme; nihayet kullanılmadı)
│       ├── trot.py                # (open-loop deneme; nihayet kullanılmadı)
│       ├── nav.py                 # NavController (Gate D; use_vy hook)
│       ├── avoider.py             # ObstacleAvoider (Gate E, harita-tabanlı)
│       ├── rangefinder_avoider.py # RangefinderAvoider (Gate E --rf, sensör)
│       └── waypoints.py           # WaypointManager (Gate F)
├── gates/                     # doğrulama betikleri (gate A–G)
│   ├── gate_b_stand.py … gate_g_g1.py
│   ├── straight_walk.py         # Gate D: hedefe git (--onnx bayrağı)
│   ├── gate_e_obstacle.py       # Gate E: engelden kaçın (--onnx --rf bayrakları)
│   └── gate_course.py           # Gate Course: 5 waypoint + 3 engel (--onnx --rf, sensör detour)
├── experiments/               # vy-policy araştırması (walk.pt/amble — reddedildi) + ONNX probe
│   └── straight_walk_onnx.py    # ONNX policy sim-to-sim transfer probe
├── scripts/                   # kurulum + sim + canlı izleme yardımcıları
│   ├── setup.sh                # tek-komut kurulum (sudo'suz; rf sahneleri + HF ONNX model indirir)
│   ├── check_env.sh            # ortam doğrula (rf sahneleri + ONNX model dahil)
│   ├── sim_headless.py         # MuJoCo + DDS bridge + rangefinder yayını (--viewer isteğe bağlı)
│   ├── use_scene.sh            # clean ↔ obstacle ↔ rf ↔ course sahne değiştir
│   ├── ros2_env.sh             # ROS2 Jazzy + DDS ortamı (bridge/runner için, §11)
│   ├── ros2_run.py             # ONNX runner ros2_cmd modu (ROS2 cmd_vel ile sürülür)
│   ├── ros2_demo_client.py     # ROS2 demo istemci (cmd_vel + /go2/stop + pose)
│   ├── ros2_demo.sh            # uçtan uca ROS2 demo (sim+runner+köprü+istemci)
│   └── stand_watch.py / walk_watch.py
├── scenes/                    # Go2 MJCF sahneleri (repo'da; setup.sh external'a kopyalar)
│   ├── go2_scene_clean.xml
│   ├── go2_scene_obstacle.xml
│   ├── go2_rangefinder.xml       # Go2 + 3 rangefinder sensörü (Gate E/Course --rf)
│   ├── go2_scene_obstacle_rf.xml # engel + rangefinder sahnesi (Gate E --rf)
│   └── go2_scene_course.xml      # 5-waypoint + 3-obstacle course (Gate Course --rf)
└── external/                  # klonlanan bağımlılıklar (.gitignore)
    ├── cyclonedds/  unitree_mujoco/  unitree_sdk2_python/
    ├── unitree-sim2real/  unitree_rl_gym/
    └── diasAiMaster_go2_velocity_flat/  # HF ONNX policy (setup.sh step 6 indirir)
```

`codeep/control/kinematics.py` ve `trot.py` (ve `gates/gate_c_walk.py`)
open-loop IK denemesinden kalmadır (§9.2); mimarinin anlaşılması için
tutulmuştur. Çekirdek düz yürüyüş `rl_runner_onnx.py` (ONNX vy policy) ile
sağlanır; `rl_runner.py` (all_gait) Gate C (ileri yürü) ve Gate F+ (waypoint)
için yedek olarak kalır.
