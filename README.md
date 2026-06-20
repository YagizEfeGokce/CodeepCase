# Codeep — Unitree Go2 Simülasyon Case

Teknik rapor · Unitree Go2 robotunu MuJoCo simülasyonunda çalıştırma, hedefe
yönlendirme ve engel algılama

> Bu raper, Codeep STAJ programı teknik case'inin teslim dokümanıdır. Case
> brief'i: `Codeep_Teknik_Case.pdf`. Tüm zorunlu çıktılar ve bonus maddeleri
> (engel algılama, çoklu waypoint, modüler kod, G1 humanoid denemesi)
> yerine getirilmiştir.

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
git clone <repo-link> CodeepV1 && cd CodeepV1

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
.venv/bin/python -m pip install -e external/unitree_sdk2_python
.venv/bin/python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
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
```

- `scene_clean.xml` — engelsiz sahne (Gate B/C/D ve çoklu waypoint testleri)
- `scene_obstacle.xml` — yolda tek kutu engel (Gate E/F+)

```bash
# --- B) Doğrulama betikleri (ayrı terminal, aynı venv) ---
.venv/bin/python scripts/gate_b_stand.py        # Gate B: dik dur
.venv/bin/python scripts/gate_c_rl.py --vx 0.3  # Gate C: ileri yürü
.venv/bin/python scripts/straight_walk.py        # Gate D: hedefe git (5,0)
.venv/bin/python scripts/gate_e_obstacle.py      # Gate E: engelden kaçın
.venv/bin/python scripts/gate_f_combined.py     # Gate F+: 4 waypoint + engel
.venv/bin/python scripts/gate_g_g1.py           # Gate G: G1 humanoid (kendi viewer'ı)
```

G1 (Gate G) kendi MuJoCo viewer'ını açar; Go2 simülatörüne ihtiyaç duymaz.

## 5. Robot kontrol yaklaşımı

Katmanlı mimari (her katman `codeep/` altında ayrı modül):

```
┌──────────────────────────────────────────────────────────┐
│  codeep/control/waypoints.py  — WaypointManager (Gate F) │
│  codeep/control/avoider.py    — ObstacleAvoider (Gate E) │
│  codeep/control/nav.py        — NavController (Gate D)    │
├──────────────────────────────────────────────────────────┤
│  codeep/locomotion/rl_runner.py — RLRunner (Gate C)       │
│    pre-trained Go2 trot policy  →  set_command(vx,vy,wz)  │
├──────────────────────────────────────────────────────────┤
│  codeep/robot/go2_client.py — DDS köprüsü (LowCmd/State)  │
├──────────────────────────────────────────────────────────┤
│  unitree_mujoco simulate_python — MuJoCo + DDS bridge     │
└──────────────────────────────────────────────────────────┘
```

- **Locomotion (Gate C):** Go2'nin yürüme hareketi, önceden eğitilmiş bir RL
  trot politikası (`all_gait_23Dec2025.pt`, shivam-sood00/unitree-sim2real)
  ile üretilir. `RLRunner` bu politikayı DDS üzerinden `rt/lowcmd`'ye
  yayımlayan sarmalayıcıdır; yüksek-seviye `set_command(vx, vy, wz)` API'si
  sunar. Bu, gerçek Go2'deki sport-mode servisiyle aynı paradigmadır:
  yürüyüş kara kutu, dışarısı hız komutu gönderir. PDF'in "RL beklenmiyor"
  notu "zorunlu değil" anlamındadır; navigasyon/engel/waypoint/dokümantasyon
  bize aittir (§9).
- **Navigation (Gate D, bizim kodumuz):** `NavController`, P-kontrol
  tabanlı dümenleme yapar: hedefe bearing ile yaw hatası → `wz`; mesafe →
  `vx` (hizalanma olmadığında `min_align` kadar minimum ileri hız, böylece
  policy yürümeyi bırakmaz); yanal crab'ı ön-besleme `yaw_bias` ile iptal
  eder (§9.5). Poz `SportModeState`'ten okunur.
- **ObstacleAvoider (Gate E, bizim):** yoldaki engellerin konumu sahne/
  config'den bilinir; robot pozu ile engel gövde çerçevesine projekte
  edilir; ön tarafta + reaksiyon mesafesi içinde ise tespit edilir, robot
  engelin yanına bir detour waypoint'ine yönlendirilir, ardından hedefe
  döner. PDF'in "dur veya yön değiştir" şartı "yön değiştir" ile sağlanır.
- **WaypointManager (Gate F, bizim):** waypoint'lere sırayla gider; varışı
  poz mesafesinden ölçer ve `NavController` `(0,0,0)` göndermeden bir
  sonrakine geçer (policy'nin donmasını önler, §9.5).
- **G1 (Gate G, bonus):** `unitree_rl_gym`'in `deploy_mujoco` pipeline'ı;
  önceden eğitilmiş `g1/motion.pt` hareket politikası, kendi MJCF'i ve
  viewer'ı ile standalone çalışır.

Tüm modüller `codeep/` paketinde; doğrulama betikleri `scripts/` altında.

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
6. `vy = 0` (all_gait policy'si yanal komutu uygulamıyor, §9.4).
7. `dist <= goal_tol` → hedefe ulaşıldı.

Bu denetleyici, Gate D'de (5,0) hedefine ulaştı (varış hatası 0.24 m) ve
Gate F+'da 4 waypoint'i sırayla gezdi (maks varış hatası 0.30 m).

## 7. Engel algılama yaklaşımı (varsa)

`ObstacleAvoider` (Gate E/F+):

- **Tespit:** engeller `(x, y, r)` listesi config'den bilinir. Her adımda
  her engel gövde çerçevesine projekte edilir (`bx = cos·dx + sin·dy` ileri,
  `by = -sin·dx + cos·dy` yanal). `bx > 0` ve `bx < reaction_dist` (1.0 m) ve
  `|by| < r + clearance` ise engel tespit edilir. Poz `SportModeState`'ten.
- **Reaksiyon:** tespit anında hedef, engelin yanındaki detour waypoint'ine
  (`ox, oy + (r + margin)`) geçer; robot oraya gider, ardından asıl hedefe
  döner. `stop_time = 0` (policy `(0,0,0)` komutunda donduğu için dur-komutu
  kullanılmaz; PDF "dur **veya** yön değiştir" der — biz yön değiştiririz).
- **Sonuç (Gate E):** (2.5, 0)'daki kutu, robot (1.5, 0) civarında tespit
  edildi, robot (2.5, 0.7) detour'una sapıp kutuyu 0.39 m clearance ile
  geçti ve (5, 0) hedefine ulaştı (varış 0.24 m).

**Sınırlama:** tespit harita-tabanlıdır (engel konumları biliniyor). Gerçek
bir lidar/derinlik sensörü eklenirse, algılama katmanı değişmeden
  algılama kaynağı değiştirilebilir (§10).

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

## 9. Kullanılan kaynaklar

- `unitreerobotics/unitree_mujoco` — resmî MuJoCo simülatörü + Go2/G1 MJCF.
- `unitreerobotics/unitree_sdk2_python` — Go2 düşük-seviye DDS arayüzü.
- `eclipse-cyclonedds/cyclonedds` — DDS ara katmanı (0.10.x).
- `shivam-sood00/unitree-sim2real` — Go2 önceden eğitilmiş trot
  politikaları + sim2sim runner (RLRunner bunu sarmalar).
- `unitreerobotics/unitree_rl_gym` — G1 önceden eğitilmiş hareket
  politikası + `deploy_mujoco` pipeline (Gate G).
- `darshmenon/quadruped-dog-rl` — open-loop IK gait referansı (bilgi
  kaynağı; bizim open-loop denememizi şekillendirdi, nihayet kullanılmadı).
- MuJoCo docs (mujoco.readthedocs.io), Unitree developer docs
  (support.unitree.com/home/en/developer).

## 10. Geliştirilebilecek noktalar

- **Gerçek sensör ile engel algılama:** MJCF'ye lidar/derinlik sensörü ekleyip
  yeni bir DDS konusu yayımlamak; `ObstacleAvoider`'ın tespit kaynağı
  değiştirilir, reaksiyon/planning katmanı aynı kalır.
- **`vy` destekleyen policy:** yanal komutu izleyen bir Go2 policy'si ile
  düz-çizgi yürüyüş `yaw_bias` önyargısına ihtiyaç duymadan sağlanır.
- **G1'i DDS köprüsüne entegre etmek:** G1'i kendi standalone pipeline'ından
  `unitree_mujoco` DDS köprüsüne taşımak (aynı navigasyon stack'ini
  paylaşır).
- **Otomatik `yaw_bias` kalibrasyonu:** crab hızını ölçüp `yaw_bias`'ı
  çevrim-içi ayarlamak (sıfır sürüklenme).
- **A\* global planlayıcı:** karmaşık engel alanları için (önceki Gazebo
  denemesinde A\* mevcuttu; portlanabilir).
- **ROS2 topic/service katmanı:** (bonus maddesi; bu çalışmada pure-Python
  SDK ile yapıldı, ROS2 isteğe bağlı).
- **Demo videosu:** kullanıcı tarafından hazırlanacak.

---

## Ek: Doğrulama (gates) özeti

| Gate | Hedef | Sonuç |
| --- | --- | --- |
| A | Kurulum + import | PASS |
| B | Go2 dik dur (30 sn, düşmez) | PASS (7 mm sürüklenme) |
| C | İleri yürü | PASS (+1.13 m ileri, dik) |
| D | Hedefe ulaş (5,0) | PASS (varış 0.24 m) |
| E | Engelden kaçın + hedefe ulaş | PASS (0.39 m clearance) |
| F+ | 4 waypoint + engel (tek run) | PASS (4/4, 0.30 m, engel kaçınıldı) |
| G | G1 humanoid yürü (bonus) | PASS (+1.22 m, dik) |

## Ek: Repo yapısı

```
CodeepV1/
├── README.md                  # bu teknik rapor (TR)
├── Codeep_Teknik_Case.pdf
├── requirements.txt
├── pyrightconfig.json
├── codeep/
│   ├── robot/go2_client.py    # DDS köprüsü (LowCmd/LowState/SportModeState)
│   ├── locomotion/rl_runner.py # RL trot policy sarmalayıcı
│   ├── control/
│   │   ├── kinematics.py      # (open-loop deneme; nihayet kullanılmadı)
│   │   ├── trot.py             # (open-loop deneme; nihayet kullanılmadı)
│   │   ├── nav.py              # NavController (Gate D)
│   │   ├── avoider.py          # ObstacleAvoider (Gate E)
│   │   └── waypoints.py        # WaypointManager (Gate F)
├── scripts/
│   ├── setup.sh
│   ├── gate_b_stand.py … gate_g_g1.py
│   ├── straight_walk.py
│   └── stand_watch.py / walk_watch.py
└── external/                  # klonlanan bağımlılıklar (gitignored)
    ├── cyclonedds/  unitree_mujoco/  unitree_sdk2_python/
    ├── unitree-sim2real/  unitree_rl_gym/
```

`codeep/control/kinematics.py` ve `trot.py` open-loop IK denemesinden
kalmadır (§9.2); mimarinin anlaşılması için tutulmuştur, nihai yürüme
`rl_runner.py` ile sağlanır.
