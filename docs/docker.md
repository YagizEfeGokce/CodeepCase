# Docker — tekrar üretilebilir ortam ("benim makinemde çalışıyor" sendromu için)

Case ortamını her cihazda aynı şekilde çalıştırmak için bir Docker imajı
sağlanır. İmaj, Ubuntu 24.04 + tüm bağımlılıkları (CycloneDDS derlemesi dahil)
içine gömülü olarak içerir; `external/` ve `.venv` kurulum sırasında imajın
içinde `scripts/setup.sh` tarafından yeniden oluşturulur.

## Hızlı kurulum (headless — X/GPU gerekmez)

```bash
docker build -t codeep .                       # ~5-10 dk (CycloneDDS derle + torch indir)
docker run --rm codeep                          # varsayılan: Gate B (dik dur) headless
docker run --rm codeep bash run.sh c --vx 0.3   # Gate C: ileri yürü
docker run --rm codeep bash run.sh f            # Gate F+: 4 waypoint + engel
docker run --rm codeep bash run.sh g1           # Gate G: G1 humanoid (bonus)
docker run --rm codeep bash scripts/check_env.sh
```

`run.sh`, `HEADLESS=1` (imajda varsayılan) iken `scripts/sim_headless.py`'yi
kullanır — GLFW viewer gerekmez, ekran/GPU olmadan fizik + DDS köprüsü çalışır.
Gate'ler DDS üzerinden her zamanki gibi bağlanır ve aynı doğrulama çıktısını
verir.

> **Konteyner doğrulandı (gerçek docker build + run):** `docker build -t codeep .`
> (RC=0) ve `docker run --rm codeep` → **Gate B PASS** (drift 0.01 m, dik);
> `docker run --rm codeep bash run.sh f` → **Gate F+ PASS** (4/4 waypoint,
> engel 0.40 m clearance). Yani tüm stack (locomotion + navigasyon + engel +
> waypoint) temiz bir konteynerde tekrar üretilebilir — "benim makinemde
> çalışıyor" sendromu yok. Host'ta da `HEADLESS=1 bash run.sh b` doğrulandı.

## GUI (MuJoCo viewer) — opsiyonel

Viewer'ı görmek için X11 ve GL gerekir. Yazılım-GL (mesa) ile:

```bash
xhost +local:docker
docker run --rm -it \
  -e DISPLAY=$DISPLAY -e HEADLESS=0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  codeep bash run.sh b
```

NVIDIA GPU ile (nvidia-container-toolkit kurulu ise, donanım hızlandırmalı GL):

```bash
xhost +local:docker
docker run --rm -it --gpus all \
  -e DISPLAY=$DISPLAY -e HEADLESS=0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  codeep bash run.sh b
```

> Not: Viewer'lı GUI, konak sürücü/GL kurulumuna bağlıdır ve bazı ekran
> kartlarında yazılım-GL ile yavaş olabilir. Case'in *doğrulaması* headless
> modda yapılır; viewer yalnızca görsel/demo amaçlıdır (demo videosu
> kullanıcı tarafından host'ta kaydedilir).

## docker-compose

```bash
docker compose up --build       # docker-compose.yml: Gate B headless
# başka gate için command geçersiz kılın (dosyaya bakın).
```

## İçerik

- `Dockerfile` — ubuntu:24.04 + sistem bağımlılıkları + `scripts/setup.sh`.
- `.dockerignore` — `external/`, `.venv/`, `.git` derleme bağlamından çıkarılır.
- `docker-compose.yml` — headless Gate B.
- `scripts/sim_headless.py` — viewersiz simülatör (headless/container/CI).
