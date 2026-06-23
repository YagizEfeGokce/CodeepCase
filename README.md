# Codeep — Unitree Go2 Simülasyon Case

Teknik rapor · Unitree Go2 robotunu MuJoCo simülasyonunda çalıştırma, hedefe
yönlendirme ve engel algılama

> Bu rapor, Codeep STAJ programı teknik case'inin teslim dokümanıdır. Case
> brief'i: `Codeep_Teknik_Case.pdf`. Zorunlu çıktıların tamamı ve **5/6 bonus
> maddesi** (engel algılama, çoklu waypoint, modüler kod, ROS2 topic/service Python wrapper şeklinde kullanılmış ama robotun etkin kullanımı kapsamamakta,
> G1 humanoid denemesi) yerine getirilmiştir.
>
> Repo: <https://github.com/YagizEfeGokce/CodeepCase>

---

## Hızlı başlangıç

```bash
git clone https://github.com/YagizEfeGokce/CodeepCase.git && cd CodeepCase
bash scripts/setup.sh          # tek komutla kurulum (sudo'suz, ~3 dk)
bash scripts/check_env.sh      # ortamı doğrula (Go2 + G1)

bash run.sh b                  # Go2 dik dur
bash run.sh c --vx 0.3         # Go2 ileri yürü
bash run.sh d --onnx           # Go2 düz yürüyüş (çekirdek) — hedefe git (5,0)
bash run.sh e --onnx --rf      # engeli sensörle algıla + yan sapıp geç
bash run.sh course --onnx --rf # 5 waypoint + 3 engel (sensörle yol bul)
bash run.sh g1                 # G1 humanoid yürü (bonus)
```

`run.sh <gate>` simülatörü doğru sahneyle başlatır, testi çalıştırır ve
kapatır — ikinci bir terminal gerekmez. Her test bir "gate" olarak adlandırılır
ve sonunda PASS/FAIL + ölçümleri yazdırır.

**Düz-çizgi yürüyüş, Go2'mizin çekirdek yeteneğidir.** ONNX tabanlı yürüyüş
politkası (`run.sh d --onnx`) ile robot, hedefe giderken yanal sapmayı
kendiliğinden düzeltir; 5 metrede sadece ~3 cm sapar ve hedefe varar.

### Docker (Her Cihazda Uyumlu Olması İçin)

Tüm ortam içine gömülü bir image — her cihazda aynı şekilde çalışır (ekran/GPU
gerekmez):

```bash
docker build -t codeep .
docker run --rm codeep                 # dik dur (headless smoke test)
docker run --rm codeep bash run.sh f   # 4 waypoint + engel (headless)
```

Ayrıntı: `docs/docker.md`.

---

## 1. Kullanılan simülasyon ortamı

| Bileşen | Seçim | Neden |
| --- | --- | --- |
| Fizik motoru | **MuJoCo** | Hızlı, doğru; Unitree'nin resmî simülatörü bunu kullanır |
| Simülatör | **unitree_mujoco** | Unitree'nin resmî simülatörü — gerçek robotla aynı arayüz |
| Haberleşme | **Cyclone DDS** | Robot ile sim arasındaki veri aktarımı (gerçek Go2'de de bu kullanılır) |
| Robot SDK | **unitree_sdk2_python** | Python'dan robota komut gönderme / durum okuma |
| OS / Python | Ubuntu 24.04 / Python 3.12 (izole `.venv`) | — |

Simülatör, MuJoCo üzerinde fizik simülasyonunu çalıştırır ve bir DDS köprüsü
aracılığıyla robotun durumunu (konum, eklem açıları, IMU) yayımlar; bizim
kodumuz da aynı köprü üzerinden robota hareket komutları gönderir. Robotun
dünya koordinatlarındaki konumu doğrudan simülatörden geldiği için ek bir SLAM
ya da konum kestirimi gerekmez.

**Neden bu ortam?** Resmî Unitree simülatörü, gerçek robotla aynı düşük seviye
arayüzü kullandığı için sim-to-real geçişi doğrudandır ve Go2'nin dik durması
/ileri yürümesi " ilk günden" çalıştı.

## 2. Kullanılan robot modeli

- **Ana robot: Unitree Go2** — 12 serbestlik dereceli (4 bacak × 3 eklem)
  dört ayaklı robot. Tüm zorunlu çıktılar ve bonuslar bu robot üzerinde
  çalışıldı.
- **Bonus robot: Unitree G1** — 29 serbestlik dereceli humanoid (iki ayaklı)
  robot. Case'in bonus maddelerinden biri olarak, kendi hareket politkasıyla
  ayrı bir deneme yapıldı (§5'te Gate G).

## 3. Kurulum adımları

Tüm dış bağımlılıklar `external/` klasörüne klonlanır; ana repo sade kalır.

```bash
git clone https://github.com/YagizEfeGokce/CodeepCase.git && cd CodeepCase
bash scripts/setup.sh     # tek komut: venv + bağımlılıklar + DDS derlemesi + modeller
bash scripts/check_env.sh # ortamı doğrula
```

`setup.sh` kabaca şunları yapar: izole bir Python sanal ortamı kurar,
CycloneDDS C kütüphanesini derler (sudo gerektirmeden), MuJoCo/SDK/policy
paketlerini kurar, Go2 sahnelerini yerine koyar ve Hugging Face üzerinden
ONNX yürüyüş politkasını indirir. Tüm adımlar sudo'suz çalışır; detaylı elle
kurulum istenirse `scripts/setup.sh` içeriğine bakılabilir.

Kurulumu doğrulamak için: `bash scripts/check_env.sh` (Go2 + G1 + modeller
hazır mı diye kontrol eder).

## 4. Çalıştırma adımları

Her test (gate) tek komutla çalışır; `run.sh` simülatörü başlatır, testi
çalıştırır ve kapatır. Sahne, testin türüne göre otomatik seçilir:

- **engelsiz sahne** — dik durma, ileri yürüyüş, hedefe gitme
- **engelli sahne** — yolda kutu engel
- **sensör sahnesi** — kutu engel + Go2'ye eklenmiş 3 mesafe sensörü
  (rangefinder), böylece robot engeli "görür"
- **kurs sahnesi** — 5 hedef nokta (waypoint) + 3 engel

Tek tek çalıştırmak isterseniz:

```bash
.venv/bin/python gates/gate_b_stand.py            # dik dur
.venv/bin/python gates/gate_c_rl.py --vx 0.3      # ileri yürü
.venv/bin/python gates/straight_walk.py --onnx    # düz yürüyüş (çekirdek) → (5,0)
.venv/bin/python gates/gate_e_obstacle.py --onnx --rf  # engeli sensörle algıla + geç
.venv/bin/python gates/gate_course.py --onnx --rf      # 5 waypoint + 3 engel
.venv/bin/python gates/gate_g_g1.py --duration 20      # G1 humanoid yürü
```

G1 kendi MuJoCo penceresini açar; Go2 simülatörüne ihtiyaç duymaz.

## 5. Robot kontrol yaklaşımı

Sistem, alttan üste doğru katmanlı bir mimariyle kuruldu. Her katman bir
sorumluluğa sahip ve bir sonrakine basit bir arayüz sunar — bu, kodun
anlaşılır ve değiştirilebilir olmasını sağlar (bonus: modüler kod yapısı).

```
┌──────────────────────────────────────────────────────────────┐
│  waypoint / engel / navigasyon katmanı (bizim kodumuz)        │
│    "şuraya git", "engel var, yana sap", "sıradaki waypoint"   │
├──────────────────────────────────────────────────────────────┤
│  yürüyüş (locomotion) katmanı                                 │
│    önceden eğitilmiş yürüyüş politikası  →  (vx, vy, wz)      │
├──────────────────────────────────────────────────────────────┤
│  robot ile sim arasındaki DDS köprüsü                         │
│    LowCmd (komut) / LowState (durum) / SportModeState (poz)   │
├──────────────────────────────────────────────────────────────┤
│  unitree_mujoco — MuJoCo fizik simülasyonu                    │
└──────────────────────────────────────────────────────────────┘
```

- **Yürüyüş katmanı:** Go2'nin yürüme hareketi, önceden eğitilmiş bir
  pekiştirmeli öğrenme (RL) politikasıyla üretilir. Dışarıdan sadece bir
  hız komutu verilir: "ileri git, bu kadar yan git, bu kadar dön" (`vx, vy,
  wz`). Bu, gerçek Go2'nin "sport mode" mantığıyla aynıdır: yürüyüş kara
  kutu, siz hızı söylersiniz. İki politika kullanıldı:
  - **ONNX vy politkası (çekirdek)** — yanal komutu (`vy`) gerçekten izler.
    Bu sayede robot hedefe giderken yanal sapmasını kendiliğinden düzeltir;
    düz yürüyüş ~3 cm sapmayla sağlanır.
  - **all_gait trot politkası (yedek)** — yanal komutu uygulamaz. Bu durumda
    yanal sürüklenme, robota önceden verilmiş bir yön ofsetiyle (`yaw_bias`)
    iptal edilir; ~18 cm sapmayla yürür.
- **Navigasyon katmanı (bizim kodumuz):** "Hedef noktaya git" mantığı
  (`NavController`). Robota "hedefe yönel, mesafeye göre hızlan, varınca dur"
  der. Hedefe giderken yanal sapmayı da düzeltir (çekirdek politkayla gerçek
  `vy` ile; yedek politkayla yön ofsetiyle).
- **Engel algılama (bizim kodumuz):** Yolda engel görünce robota "engelin
  yanına sap, geç, sonra hedefe dön" der. İki modu var: engel konumunu
  önceden bilmek (harita) ya da sensörle görmek (§7).
- **Waypoint yöneticisi (bizim kodumuz):** Birden fazla hedefe sırayla götürür.
- **G1 (bonus):** G1 humanoid, kendi hareket politkasıyla ayrı bir pipeline
  olarak çalışır (Go2 DDS köprüsünü kullanmaz).

Tüm modüller `codeep/` paketinde; test betikleri `gates/` altında.

## 6. Hedef noktaya yönlendirme mantığı

"Robotu başlangıç noktasından belirlenen hedefe götür" zorunlu maddesi,
`NavController` ile sağlanır. Mantık basit bir kapalı çevrim (geri beslemeli)
denetleyicidir; her döngüde şu soruları yanıtlar:

1. **Neredeyim?** Robotun konumu ve yönü (yaw) simülatörden okunur.
2. **Hedef nerede, hangi yönde?** Hedefe olan vektör hesaplanır; istenen
   yön bulunur.
3. **Yön hatası ne kadar?** Robotun yönü ile hedef yönü arasındaki fark
   (`yaw_err`) hesaplanır. Bu fark, dönüş hızını (`wz`) belirler — ne kadar
   sapmışsa o kadar hızlı döner.
4. **Ne kadar hızlanayım?** Hedefe olan mesafe, ileri hızı (`vx`) belirler.
   Hedefe yakınlaştıkça yavaşlar. Henüz hedefe dönük değilse bile çok
   yavaşlamaz (yoksa yürüyüş durar) — minimum bir ileri hız korunur.
5. **Yanal sapmayı düzelteyim:** Çekirdek politka ile robota yanal bir hız
   (`vy`) gönderilir ve hedef çizgisinde kalması sağlanır. Yedek politkada
   `vy` çalışmadığı için yön ofseti (`yaw_bias`) ile bu sürüklenme önceden
   iptal edilir.
6. **Vardım mı?** Mesafe, toleransın altına düşünce hedefe ulaşılmış sayılır.

**Sonuç:** Gate D'de (5,0) hedefine ulaşıldı — çekirdek politkayla varış
hatası 24 cm ve **maksimum yanal sapma sadece 3 cm** (yön ofseti yok). Yedek
politkayla da ulaşıldı ama yanal sapma ~18 cm. Gate F+'da 4 waypoint sırayla
gezildi (maks varış hatası 30 cm).

## 7. Engel algılama yaklaşımı (varsa)

Case'in bonus maddesi: "simülasyona basit bir engel ekle ve robot engeli
algıladığında dursun ya da yön değiştirsin." Biz **yön değiştir** (engelin
etrafından dolaş) dedik. İki mod var; ikisi de aynı mantığı kullanır —
"engel görürse yan taraftaki bir noktaya sap, geç, sonra asıl hedefe dön" —
sadece **engeli nasıl gördüğü** farklı.

### 7.1 Sensörle algılama (bonus yol, `--rf`)

- **Sensör:** Go2'nin gövdesine 3 mesafe sensörü (rangefinder) eklendi:
  öne, ön-sola ve ön-sağa bakan. Simülatör her adımda bu sensörlerin
  ölçümlerini yayınlar. Böylece robot engelin konumunu **bilmez**, onu
  sensörle **görür**.
- **Algılama:** Ön sensör bir şeyin yakın mesafede olduğunu söyleyince
  engel tespit edilir. Sol ve sağ sensörlerden hangisi daha "boş" görüyorsa,
  robot o tarafa sapar.
- **Geçiş:** Robot, engelin yanındaki bir noktaya gider. **Önemli detay:**
  bu noktaya gerçekten ulaşmadan asıl hedefe dönmez — burnunu çevirip "önüm
  açıldı" diye düşünüp hemen hedefe dönseydi, tekrar engelin yoluna girerdi.
  Yan noktaya ulaşıyor, sonra hedefe dönüyor; yolda yeni bir engel görürse
  yine sapıyor.
- **Sonuç (Gate E `--onnx --rf`):** ~1.2 m'den engeli gördü, sola sapıp
  kutuyu 42 cm boşlukla geçti ve (5,0) hedefine ulaştı (24 cm varış, çarpmadı,
  düşmedi).

### 7.2 Haritadan algılama (varsayılan)

Engellerin konumu sahneden önceden bilinir. Robot, kendi konumuna göre
engelin "önümde mi, yanımda mı" olduğunu hesaplar; önündeyse yan noktaya
sapar. Sonuç yine aynı: engeli 39 cm boşlukla geçip hedefe ulaştı.

> Sensör modu, §10'daki "gerçek sensörle engel algılama" hedefini gerçekleştirir.
> İki modun reaksiyon/planning katmanı aynı; sadece tespitin kaynağı değişir.

## 8. Karşılaşılan problemler ve çözüm yaklaşımı

1. **Go2 ilk denemede ayağa kalkamıyordu.** Gazebo/ROS2 ortamında robot düz
   bacaklarla doğup düşüyordu. *Çözüm:* resmî Unitree simülatörüne geçtik;
   burada duruş pozu doğrudan komutlanabiliyor ve Go2 dik duruyor (30 sn, 7 mm
   sürüklenme).
2. **Kendi yazdığımız yürüyüş (open-loop) yürümedi.** Bacaklar yerde kayıyor,
   gövde istenen yönün tersine sürükleniyordu. *Çözüm:* yürüyüşü önceden
   eğitilmiş bir RL politkasına bıraktık; navigasyonu biz yazdık. (Case "gelişmiş
   RL beklenmiyor" diyor; biz yine de önceden eğitilmiş politka kullandık.)
3. **Yürüyüş politkası yanal komutu (`vy`) uygulamıyordu.** Bu politka ile
   robot "yan git" denince yan gitmiyor, öne gidiyordu. Bu yüzden düz
   yürüyüşte sağa kayıyordu. *Çözüm (ilk):* yön komutuna sabit bir ofset
   (`yaw_bias`) ekleyerek bu kaymayı önceden iptal ettik (~18 cm sapma).
   *Çözüm (nihai):* yanal komutu gerçekten izleyen ONNX politkasına geçtik;
   artık ofset gerekmiyor, sapma ~3 cm'ye düştü.
4. **Robot "dur" komutunda donuyordu.** Sıfır komut gönderilince yürüyüş
   politkası tamamen duruyor ve bir daha başlamıyordu. *Çözüm:* "dur" komutu
   kullanmadık; waypoint varışını konum mesafesinden ölçüp, robot tam
   durmadan bir sonraki hedefe geçirdik.
5. **90° dönüşlerde robot yere basıp kalıyordu.** Dönüşte tamamen durunca
   yürüyüş bozuluyordu. *Çözüm:* hizalanmamış olsa bile minimum bir ileri hız
   koruduk; böylece robot yürüyerek dönüyor.
6. **Mesafe sensörü engeli görmüyordu.** Sensör göğüs yüksekliğindeydi, engel
   ise çok kısaydı; yatay ışın engelin üzerinden geçiyordu. *Çözüm:* engeli
   biraz daha yüksek yaptık, böylece sensör görebildi.
7. **Robot engeli görüp sapanca, hemen hedefe dönüp engelle çarpıyordu.**
   Burnu yana dönünce "önüm açıldı" sanıp erken hedefe dönüyordu. *Çözüm:*
   yan noktaya gerçekten ulaşmadan hedefe dönmesi engellendi; yolda yeni engel
   görürse tekrar sapması eklendi.

## 9. Kullanılan kaynaklar

- `unitreerobotics/unitree_mujoco` — resmî MuJoCo simülatörü + Go2/G1 modelleri.
- `unitreerobotics/unitree_sdk2_python` — Go2 düşük seviye DDS arayüzü.
- `eclipse-cyclonedds/cyclonedds` — DDS haberleşme katmanı.
- `shivam-sood00/unitree-sim2real` — Go2 için önceden eğitilmiş trot yürüyüş
  politkası (yedek).
- `diasAiMaster/unitree-go2-velocity-flat` (Hugging Face) — Go2 için önceden
  eğitilmiş vy destekli yürüyüş politkası (ONNX, çekirdek).
- `unitreerobotics/unitree_rl_gym` — G1 humanoid için önceden eğitilmiş hareket
  politkası (Gate G).
- ROS2 Jazzy + `rclpy` — ROS2 köprüsü için (§11).
- MuJoCo ve Unitree dokümantasyonları.

## 10. Geliştirilebilecek noktalar

- **ROS2 topic/service katmanı** — Bütün komutların ROS2 service'ine geçirilmesi(bonus, §11).
- **G1'i DDS köprüsüne entegre etmek:** G1'i kendi standalone pipeline'ından
  alıp Go2'nin DDS köprüsüne taşımak (aynı navigasyon mantığını paylaşır).
- **Daha akıllı yol planlama (A\*):** Şu anki engel kaçınma "tek tek engel
  görünce yan sap" mantığıyla (reaktif) çalışıyor. Karmaşık/labirent ortamlar
  için A* gibi global bir planlayıcı eklenebilir.
- **Daha geniş sensör taraması:** 3 sabit ışın yerine dönen lidar ya da
  derinlik kamerası ekleyerek engel tespit açısını büyütmek.

## 11. ROS2 topic/service köprüsü (bonus)

Case'in "ROS2 topic/service yapısının etkin kullanılması" bonus maddesi.
Mevcut DDS arayüzünü, standart ROS2 topic ve service'lerine açan bir köprü
düğümü yazdık (`codeep/ros2_bridge.py`, ROS2 Jazzy üzerinde). Böylece Go2,
sıradan bir ROS2 stack'inden komutlanabilir ve durumu ROS2 topic'leri olarak
okunabilir.

- **Durum → ROS2 (telemetri):** robotun konumu (`/go2/pose`), IMU'su
  (`/go2/imu`), eklem durumları (`/go2/joint_states`) ve mesafe sensörleri
  (`/go2/range_*`) ROS2 topic'leri olarak yayımlanır.
- **ROS2 → komut:** standart `cmd_vel` topic'i (`/go2/cmd_vel`) ile robota
  "ileri/yan/dön" hızı gönderilir; `/go2/stop` servisi ile robot durdurulur.

Yürüyüş politkası, ROS2 `cmd_vel` komutunu DDS üzerinden okuyup robotu
sürer. Yani robotu `ros2 topic pub /go2/cmd_vel ...` ile sürebilir,
`ros2 service call /go2/stop ...` ile durdurabilirsiniz.

```bash
bash scripts/ros2_demo.sh   # sim + yürüyüş + köprü + demo istemci (uçtan uca)
# veya tek tek:
source scripts/ros2_env.sh
.venv/bin/python scripts/ros2_run.py            # yürüyüş politkası (ROS2 cmd_vel'i dinler)
python3 codeep/ros2_bridge.py                   # DDS <-> ROS2 köprü düğümü
ros2 topic pub /go2/cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}"   # sür
ros2 service call /go2/stop std_srvs/Trigger                            # dur
ros2 topic echo /go2/pose                                                # konumu gör
```

**Doğrulama:** demo istemci `cmd_vel` ile robotu 6 sn ileri sürdü (0.01 m'den
1.78 m'ye, ~0.3 m/s), `/go2/stop` servisi robotu durdurdu (konum sabit 1.78 m),
`/go2/pose` telemetrisi okundu. Yani ROS2 topic + service yolu uçtan uca
çalışıyor.

---

## Ek: Doğrulama (gates) özeti

| Gate | Hedef | Sonuç |
| --- | --- | --- |
| A | Kurulum + import | PASS |
| B | Go2 dik dur (30 sn, düşmez) | PASS (7 mm sürüklenme) |
| C | İleri yürü | PASS (+1.13 m ileri, dik) |
| D `--onnx` | **Çekirdek** düz yürüyüş (5,0) — ONNX vy policy | PASS (varış 0.24 m, **max yanal 0.03 m**) |
| D | Hedefe ulaş (5,0) — all_gait + yaw_bias (yedek) | PASS (varış 0.24 m, ~0.18 m yanal) |
| E | Engelden kaçın + hedefe ulaş (harita-tabanlı) | PASS (0.39 m boşluk) |
| E `--onnx --rf` | Engelden kaçın — **sensörle** tespit | PASS (0.42 m boşluk, 0.24 m varış, düşmedi) |
| F+ | 4 waypoint + engel (tek run) | PASS (4/4, 0.30 m, engel kaçınıldı) |
| Course `--onnx --rf` | 5 waypoint + 3 engel — **sensörle** çoklu-engel (bonus) | PASS (5/5 wp, 4 sensör detour, 0.33 m boşluk, düşmedi) |
| G | G1 humanoid yürü (bonus) | PASS (+1.50 m ileri, dik) |
| ROS2 | DDS<->ROS2 köprü (cmd_vel + stop + telemetri) | PASS (cmd_vel ile yürüdü, stop ile durdu) |

## Ek: Repo yapısı

```
CodeepCase/
├── README.md                  # bu teknik rapor
├── Codeep_Teknik_Case.pdf     # case brief'i
├── requirements.txt
├── codeep/                    # kütüphane (katmanlı mimari)
│   ├── robot/                 # DDS köprüsü + IDL tipleri (rangefinder, cmd_vel)
│   ├── ros2_bridge.py         # DDS <-> ROS2 köprü düğümü (§11, bonus)
│   ├── locomotion/            # yürüyüş politkaları (ONNX çekirdek + all_gait yedek)
│   └── control/               # navigasyon, engel algılama, waypoint yöneticisi
├── gates/                     # doğrulama testleri (gate A–G + course)
├── experiments/               # yürüyüş politka araştırması (reddedilen denemeler + ONNX probe)
├── scripts/                   # kurulum, sim, sahne değiştirme, ROS2 demo, canlı izleme
├── scenes/                    # Go2 MJCF sahneleri (engelsiz / engelli / sensör / kurs)
└── external/                  # klonlanan bağımlılıklar + indirilen modeller (.gitignore)
```

`codeep/control/` içindeki `kinematics.py` ve `trot.py`, ilk open-loop yürüyüş
denemesinden kalmadır ve mimarinin anlaşılması için tutulmuştur; nihai yürüyüş
`locomotion/` içindeki politkalarla sağlanır.
