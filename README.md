# Repairman Robot Pipeline

Bu repo, ROS 2 Jazzy tabanli kapali dongu bir onarim (repair) pipeline'idir.
Sistem; goruntu alir, hasari tespit eder, toolpath uretir, onarimi simule eder,
yeniden olcup kalite skoruna gore bir sonraki aksiyona karar verir.

## 1. Proje Ne Hakkinda?

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

## 2. Calisma Ortami

Proje Docker ile konteynerize edilmis durumda.

- Compose dosyasi: `docker-compose.yml`
- Servis adi: `repairman`
- Container name: `repairman_ros2`
- Dockerfile: `docker/Dockerfile`
- Workspace mount: `./:/workspaces/repairman_ws`

Lokal calistirma da mumkun, fakat takim ici tekrar uretilebilirlik icin Docker
yontemi onerilir.

## 3. Su Ana Kadar Tamamlanan Phase'ler

Asagidaki fazlar, kod ve dokuman tarafinda tamamlanmistir.

### Phase 0 - Workspace ve Altyapi Kurulumu (Tamamlandi)

- ROS 2 Jazzy tabanli paket yapisi kuruldu.
- `colcon` build akisi olusturuldu.
- Docker image/compose tanimlari eklendi.

### Phase 1 - Perception ve Planning Hatti (Tamamlandi)

- `image_pub` ile test datasindan surekli kamera yayini.
- `yolo_node` ile hasar bolgesi tespiti ve mask uretimi.
- `toolpath_node` ile maskeden polygon tabanli yol uretimi.

### Phase 2 - Closed-Loop Repair Mantigi (Tamamlandi, MVP)

- `execute_node` ile onarim yuruyusu simulasyonu.
- `verify_node` ile kalite metrik hesaplama (before/after).
- `state_manager` ile otomatik state gecisleri ve retry karari.

### Phase 3 - Orkestrasyon ve Gorsellestirme (Tamamlandi)

- `launch/launch.py` ile tum node'lar tek komutta ayağa kalkiyor.
- RViz entegrasyonu, TF bridge ve demo robot publish eklendi.
- Parametrelesmis launch argumanlari tanimlandi.

### Phase 4 - Dokumantasyon ve Operasyon (Tamamlandi)

- `README_REPAIRMAN.md`: detayli teknik rehber
- `QUICKSTART.md`: 5 dakikalik hizli baslangic
- `BUILD_DEPLOY_COMMANDS.md`: kopyala-calistir komutlar
- `verify_setup.sh`, `quick_test.sh`, `test_pipeline.sh`: operasyon scriptleri

## 4. Mevcut Durum Ozeti (2026-02-16)

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

## 5. Projeyi Calistirma

### 5.1 Docker ile (Onerilen)

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

### 5.2 Hizli Dogrulama

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

## 6. Takim Paylasimi Icin Onerilen Kullanim

- Teknik derinlik icin: `README_REPAIRMAN.md`
- Hemen calisma icin: `QUICKSTART.md`
- Operasyon komutlari icin: `BUILD_DEPLOY_COMMANDS.md`
- Bu dosya (`README.md`) ise proje yonetsel özeti + phase durumu icindir.

## 7. Bundan Sonra Yapilmasi Gerekenler (Roadmap)

Asagidaki sirayla ilerlemek, takima devir ve stabil calisma icin en dusuk riskli
yoldur.

### Phase 5 - Stabilizasyon (Oncelik: Yuksek)

1. `verify_setup.sh` OpenCV kontrolunu `cv2` ile duzelt.
2. `quick_test.sh` topic sayim/parsing ve fail-fast davranisini duzelt.
3. `ultralytics` ve agir bagimliliklar icin net kurulum/profil ayrimi ekle
   (CPU/GPU profilleri).
4. Lint kurallari ile kod stil borcunu azalt (kademeli).

### Phase 6 - CI/CD ve Kalite Kapilari (Oncelik: Yuksek)

1. GitHub Actions:
   - build
   - unit test
   - lint
2. PR template + branch korumalari.
3. Release tagging ve changelog standartlari.

### Phase 7 - Gercek Donanim Entegrasyonu (Oncelik: Orta)

1. `image_pub` yerine gercek kamera driver'i.
2. `execute_node` yerine robot kontrol arayuzu (action/service).
3. Extruder/repair tool kontrol katmani.
4. Kalibrasyon (camera-robot frame alignment) ve tolerans testleri.

### Phase 8 - Uretim Hazirligi (Oncelik: Orta)

1. Parametre setlerinin environment bazli ayrimi.
2. Telemetry/metrics/log aggregation.
3. Hata senaryolari icin recovery stratejileri.

## 8. Kisa Komut Ozeti

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

## 9. Lisans ve Kaynaklar

- Lisans: `MIT` (`src/repairman_vision/package.xml`)
- Ana teknik dokuman: `README_REPAIRMAN.md`
