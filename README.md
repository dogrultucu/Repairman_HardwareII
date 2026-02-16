# Repairman Robot Pipeline

## EN

This repository contains a ROS 2 Jazzy based closed-loop repair pipeline.
The system captures an image, detects damage, generates a toolpath, simulates
repair execution, and decides the next action based on a quality score.

### 1. What Is This Project About?

Goal: demonstrate an automated end-to-end repair loop for damaged surfaces
inside a single ROS 2 pipeline.

Flow:

`SCAN -> DETECT -> PLAN -> REPAIR -> RESCAN -> EVALUATE -> PASS/RETRY`

Main package:

- `src/repairman_vision`

Main nodes:

- `image_pub`: fake camera publisher from test image
- `yolo_node`: YOLO/dummy damage mask generation
- `toolpath_node`: converts mask to polygon/toolpath
- `execute_node`: simulates toolpath execution
- `verify_node`: computes quality score from before/after masks
- `state_manager`: controls the full process with a state machine
- `robot_description_pub`: publishes robot description for RViz

### 2. Runtime Environment

The project is containerized with Docker.

- Compose file: `docker-compose.yml`
- Service name: `repairman`
- Container name: `repairman_ros2`
- Dockerfile: `docker/Dockerfile`
- Workspace mount: `./:/workspaces/repairman_ws`

Local native execution is also possible, but Docker is recommended for team
reproducibility.

### 3. Completed Phases So Far

The following phases are completed in code and documentation.

#### Phase 0 - Workspace and Infrastructure Setup (Completed)

- ROS 2 Jazzy package structure created.
- `colcon` build workflow prepared.
- Docker image/compose definitions added.

#### Phase 1 - Perception and Planning Pipeline (Completed)

- Continuous fake camera publishing via `image_pub`.
- Damage detection and mask generation via `yolo_node`.
- Polygon-based path generation from mask via `toolpath_node`.

#### Phase 2 - Closed-Loop Repair Logic (Completed, MVP)

- Simulated repair execution via `execute_node`.
- Quality metric computation via `verify_node`.
- Automatic state transitions and retry logic via `state_manager`.

#### Phase 3 - Orchestration and Visualization (Completed)

- Single-command startup with `launch/launch.py`.
- RViz integration, TF bridge, and demo robot publishing added.
- Parameterized launch arguments defined.

#### Phase 4 - Documentation and Operations (Completed)

- `README_REPAIRMAN.md`: detailed technical guide
- `QUICKSTART.md`: 5-minute quick start
- `BUILD_DEPLOY_COMMANDS.md`: copy-paste operations guide
- `verify_setup.sh`, `quick_test.sh`, `test_pipeline.sh`: operations scripts

### 4. Current Status Summary (2026-02-16)

The codebase is MVP-functional, but there are still improvements needed before
production-level team handover:

1. When `ultralytics` is missing, `yolo_node` falls back to dummy mode.
2. `verify_setup.sh` checks `opencv` import; this should be `cv2` to avoid
   false warnings.
3. `quick_test.sh` has integer parsing issues in topic counting and may report
   misleading success.
4. Lint/format technical debt is still high (flake8/pep257 rule violations).

Note: in restricted/sandboxed environments, DDS/RTPS permission errors may
appear. These can be environment-related rather than code-related.

### 5. How to Run the Project

#### 5.1 With Docker (Recommended)

```bash
cd ~/repairman_ws
docker compose up -d --build
docker exec -it repairman_ros2 bash
```

Inside container:

```bash
cd /workspaces/repairman_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select repairman_vision
source install/setup.bash
ros2 launch repairman_vision launch.py
```

#### 5.2 Quick Validation

In a new terminal:

```bash
cd ~/repairman_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 bag record -a -o full_pipeline_bag
```

After 30-60 seconds:

```bash
ros2 bag info full_pipeline_bag
```

Expected topics:

- `/camera/image_raw`
- `/damage/mask`
- `/damage/annotated`
- `/repair/toolpath`
- `/repair/executed`
- `/repair/quality_score`

### 6. Recommended Team Usage

- For technical depth: `README_REPAIRMAN.md`
- For fast bring-up: `QUICKSTART.md`
- For command cookbook: `BUILD_DEPLOY_COMMANDS.md`
- This file (`README.md`) is the management-level summary + phase status.

### 7. Next Steps (Roadmap)

Following this sequence is the lowest-risk path for team transfer and
stabilization.

#### Phase 5 - Stabilization (High Priority)

1. Fix OpenCV check in `verify_setup.sh` (`cv2` import).
2. Fix topic count parsing and fail-fast behavior in `quick_test.sh`.
3. Clarify dependency profiles for heavy packages (`ultralytics`, CPU/GPU).
4. Reduce style/lint technical debt incrementally.

#### Phase 6 - CI/CD and Quality Gates (High Priority)

1. Add GitHub Actions:
   - build
   - unit test
   - lint
2. Add PR template and branch protection rules.
3. Standardize release tagging and changelog workflow.

#### Phase 7 - Real Hardware Integration (Medium Priority)

1. Replace `image_pub` with real camera driver.
2. Replace `execute_node` with robot control interface (action/service).
3. Add extruder/repair-tool control layer.
4. Add calibration and frame alignment validation.

#### Phase 8 - Production Readiness (Medium Priority)

1. Environment-specific parameter sets.
2. Telemetry/metrics/log aggregation.
3. Recovery strategies for failure scenarios.

### 8. Quick Command Summary

```bash
# Setup verification
bash verify_setup.sh

# Quick diagnostics
bash quick_test.sh

# Full test pipeline
bash test_pipeline.sh

# Main launch
ros2 launch repairman_vision launch.py
```

### 9. License and References

- License: `MIT` (`src/repairman_vision/package.xml`)
- Main technical documentation: `README_REPAIRMAN.md`

---

## TR

Bu repo, ROS 2 Jazzy tabanli kapali dongu bir onarim (repair) pipeline'idir.
Sistem; goruntu alir, hasari tespit eder, toolpath uretir, onarimi simule eder,
yeniden olcup kalite skoruna gore bir sonraki aksiyona karar verir.

### 1. Proje Ne Hakkinda?

Amaç: Endustriyel bir yuzeyde hasar tespitinden sonra otomatik onarim dongusunu
tek bir ROS 2 pipeline'i icinde gostermek.

Akis:

`SCAN -> DETECT -> PLAN -> REPAIR -> RESCAN -> EVALUATE -> PASS/RETRY`

Ana paket:

- `src/repairman_vision`

Ana node'lar:

- `image_pub`: test goruntusu ile sahte kamera yayini
- `yolo_node`: YOLO/dummy hasar maskesi uretimi
- `toolpath_node`: maske -> polygon/toolpath
- `execute_node`: toolpath yurutme simulasyonu
- `verify_node`: before/after mask ile kalite skoru
- `state_manager`: tum sureci state machine ile yonetir
- `robot_description_pub`: RViz icin robot model aciklamasi

### 2. Calisma Ortami

Proje Docker ile konteynerize edilmis durumda.

- Compose dosyasi: `docker-compose.yml`
- Servis adi: `repairman`
- Container name: `repairman_ros2`
- Dockerfile: `docker/Dockerfile`
- Workspace mount: `./:/workspaces/repairman_ws`

Lokal calistirma da mumkun, fakat takim ici tekrar uretilebilirlik icin Docker
yontemi onerilir.

### 3. Su Ana Kadar Tamamlanan Phase'ler

Asagidaki fazlar, kod ve dokuman tarafinda tamamlanmistir.

#### Phase 0 - Workspace ve Altyapi Kurulumu (Tamamlandi)

- ROS 2 Jazzy tabanli paket yapisi kuruldu.
- `colcon` build akisi olusturuldu.
- Docker image/compose tanimlari eklendi.

#### Phase 1 - Perception ve Planning Hatti (Tamamlandi)

- `image_pub` ile test datasindan surekli kamera yayini.
- `yolo_node` ile hasar bolgesi tespiti ve mask uretimi.
- `toolpath_node` ile maskeden polygon tabanli yol uretimi.

#### Phase 2 - Closed-Loop Repair Mantigi (Tamamlandi, MVP)

- `execute_node` ile onarim yuruyusu simulasyonu.
- `verify_node` ile kalite metrik hesaplama (before/after).
- `state_manager` ile otomatik state gecisleri ve retry karari.

#### Phase 3 - Orkestrasyon ve Gorsellestirme (Tamamlandi)

- `launch/launch.py` ile tum node'lar tek komutta ayağa kalkiyor.
- RViz entegrasyonu, TF bridge ve demo robot publish eklendi.
- Parametrelesmis launch argumanlari tanimlandi.

#### Phase 4 - Dokumantasyon ve Operasyon (Tamamlandi)

- `README_REPAIRMAN.md`: detayli teknik rehber
- `QUICKSTART.md`: 5 dakikalik hizli baslangic
- `BUILD_DEPLOY_COMMANDS.md`: kopyala-calistir komutlar
- `verify_setup.sh`, `quick_test.sh`, `test_pipeline.sh`: operasyon scriptleri

### 4. Mevcut Durum Ozeti (2026-02-16)

Kod tabani MVP olarak calisir durumda; ancak paylasim oncesi iyilestirme gereken
noktalar vardir:

1. `ultralytics` eksik oldugunda `yolo_node` dummy moda dusuyor.
2. `verify_setup.sh` icinde OpenCV import kontrolu `opencv` yerine `cv2`
   olmali; su an yanlis alarm uretebiliyor.
3. `quick_test.sh` topic sayiminda integer parse problemi var; script bazen
   "basarili" bitse de gercek saglik durumunu dogru raporlamiyor.
4. Lint tarafinda (flake8/pep257 kurallari) teknik borc yuksek.

Not: Kisitli/sandbox ortamlarda DDS/RTPS izin hatalari gorulebilir; bu durum
koddan ziyade calistirma ortami kaynakli olabilir.

### 5. Projeyi Calistirma

#### 5.1 Docker ile (Onerilen)

```bash
cd ~/repairman_ws
docker compose up -d --build
docker exec -it repairman_ros2 bash
```

Container icinde:

```bash
cd /workspaces/repairman_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select repairman_vision
source install/setup.bash
ros2 launch repairman_vision launch.py
```

#### 5.2 Hizli Dogrulama

Yeni terminal:

```bash
cd ~/repairman_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 bag record -a -o full_pipeline_bag
```

30-60 sn sonra:

```bash
ros2 bag info full_pipeline_bag
```

Beklenen topic'ler:

- `/camera/image_raw`
- `/damage/mask`
- `/damage/annotated`
- `/repair/toolpath`
- `/repair/executed`
- `/repair/quality_score`

### 6. Takim Paylasimi Icin Onerilen Kullanim

- Teknik derinlik icin: `README_REPAIRMAN.md`
- Hemen calisma icin: `QUICKSTART.md`
- Operasyon komutlari icin: `BUILD_DEPLOY_COMMANDS.md`
- Bu dosya (`README.md`) ise proje yonetsel ozeti + phase durumu icindir.

### 7. Bundan Sonra Yapilmasi Gerekenler (Roadmap)

Asagidaki sirayla ilerlemek, takima devir ve stabil calisma icin en dusuk riskli
yoldur.

#### Phase 5 - Stabilizasyon (Oncelik: Yuksek)

1. `verify_setup.sh` OpenCV kontrolunu `cv2` ile duzelt.
2. `quick_test.sh` topic sayim/parsing ve fail-fast davranisini duzelt.
3. `ultralytics` ve agir bagimliliklar icin net kurulum/profil ayrimi ekle
   (CPU/GPU profilleri).
4. Lint kurallari ile kod stil borcunu azalt (kademeli).

#### Phase 6 - CI/CD ve Kalite Kapilari (Oncelik: Yuksek)

1. GitHub Actions:
   - build
   - unit test
   - lint
2. PR template + branch korumalari.
3. Release tagging ve changelog standartlari.

#### Phase 7 - Gercek Donanim Entegrasyonu (Oncelik: Orta)

1. `image_pub` yerine gercek kamera driver'i.
2. `execute_node` yerine robot kontrol arayuzu (action/service).
3. Extruder/repair tool kontrol katmani.
4. Kalibrasyon (camera-robot frame alignment) ve tolerans testleri.

#### Phase 8 - Uretim Hazirligi (Oncelik: Orta)

1. Parametre setlerinin environment bazli ayrimi.
2. Telemetry/metrics/log aggregation.
3. Hata senaryolari icin recovery stratejileri.

### 8. Kisa Komut Ozeti

```bash
# Setup dogrulama
bash verify_setup.sh

# Hizli test
bash quick_test.sh

# Full test akisi
bash test_pipeline.sh

# Ana launch
ros2 launch repairman_vision launch.py
```

### 9. Lisans ve Kaynaklar

- Lisans: `MIT` (`src/repairman_vision/package.xml`)
- Ana teknik dokuman: `README_REPAIRMAN.md`
