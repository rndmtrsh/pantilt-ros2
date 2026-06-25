# Pan Tilt Controller (ROS 2)

Dokumentasi ini merangkum struktur sistem `pan_tilt_controller` secara ringkas dan siap dipakai untuk halaman GitHub repository.

## 1) Deskripsi Sistem

Sistem ini adalah pipeline kontrol pan-tilt berbasis ROS 2 untuk melakukan tracking manusia menggunakan model YOLO (`best.pt`, class `Human-body`) dari kamera lalu menggerakkan aktuator pan/tilt melalui serial.

Alur utama:
1. Node visi membaca frame kamera, menjalankan YOLO untuk target `Human-body`, lalu menghitung error posisi target terhadap pusat frame.
2. Node PID mengubah error posisi menjadi perintah kecepatan pan/tilt.
3. Node serial mengirim perintah tersebut ke mikrokontroler (mis. STM32/ESP32) via UART.
4. (Opsional) Node `metrics_logger` mencatat `/vision_error`, `/cmd_vel`, dan latency ke file CSV.

Tujuan kontrol: menjaga target tetap dekat titik tengah frame kamera.

## 2) Arsitektur Node

Node yang dijalankan dari launch file `launch/pan_tilt.launch.py`:

| Node Name (runtime) | Executable | Peran |
|---|---|---|
| `/camera_vision` | `camera_vision_node` | Deteksi manusia (YOLO), hitung error X/Y, tampilkan window debug OpenCV |
| `/pid` | `pid_node` | Hitung kontrol PID dari error visi menjadi `cmd_vel` |
| `/serial` | `serial_node` | Kirim `cmd_vel` ke perangkat melalui serial |
| `/metrics_logger` | `metrics_logger` | Logging CSV untuk `/vision_error`, `/cmd_vel`, dan latency (opsional) |

Catatan:
- Nama runtime di atas berasal dari `name=` pada launch file.
- Nama class pada source adalah `CameraVisionNode`, `PIDNode`, `SerialNode`, `MetricsLogger`.

## 3) Parameter List

### 3.1 `/camera_vision`

| Parameter | Tipe | Default (kode) | Default (launch) | Keterangan |
|---|---|---|---|---|
| `camera_id` | integer | `0` | `0` | Index kamera OpenCV |
| `frame_width` | integer | `854` | `854` | Lebar frame (pixel) |
| `frame_height` | integer | `480` | `480` | Tinggi frame (pixel) |
| `deadzone_h` | float | `0.40` | `0.50` | Deadzone horizontal (rasio 0-1 dari frame width) |
| `deadzone_v` | float | `0.40` | `0.40` | Deadzone vertikal (rasio 0-1 dari frame height) |
| `show_debug` | boolean | `False` | `true` | Tampilkan window debug OpenCV |
| `flip_horizontal` | boolean | `True` | `True` | Mirror frame horizontal |
| `camera_backend` | string | `v4l2` | (default) | Backend kamera OpenCV |
| `capture_rate` | float | `30.0` | `24.0` | Frekuensi capture frame (Hz) |
| `inference_rate` | float | `10.0` | `10.0` | Frekuensi inferensi YOLO (Hz) |
| `max_jump` | integer | `200` | `500` | Threshold jarak maksimal jump filter (pixel) |
| `reacquire_timeout` | float | `0.5` | `0.5` | Timeout untuk reacquire target (detik) |
| `vertical_ref_ratio` | float | `0.33` | `0.33` | Posisi vertikal reference untuk ROI (rasio dari atas) |

Catatan: model YOLO `best.pt` dibaca dari folder paket, target class `Human-body` bersifat hardcoded. Deadzone terpisah untuk horizontal dan vertikal memungkinkan kontrol yang lebih presisi. Jump filter mencegah deteksi melompat/flicker.

### 3.2 `/pid`

| Parameter | Tipe | Default (kode) | Default (launch) | Keterangan |
|---|---|---|---|---|
| `Kp` | float | `0.5` | `2.1` | Gain proporsional |
| `Ki` | float | `0.01` | `0.8` | Gain integral |
| `Kd` | float | `0.1` | `1.2` | Gain derivatif |
| `max_vel` | float | `2000` | `5000` | Batas saturasi output (unit: vu/detik dari mikrokontroler) |
| `accel_limit` | float | `2500.0` | `1000.0` | Batas akselerasi (vel/detik) |
| `control_rate` | float | `20` | `20` | Frekuensi loop kontrol (Hz) |

Catatan: Launch file mendefinisikan parameter runtime aktual. Anti-windup diterapkan saat saturasi, integrator direset saat target hilang atau dalam deadzone.

### 3.3 `/serial`

| Parameter | Tipe | Default (kode) | Default (launch) | Keterangan |
|---|---|---|---|---|
| `port` | string | `/dev/ttyACM0` | `/dev/ttyACM0` | Port serial perangkat |
| `baudrate` | integer | `115200` | `115200` | Kecepatan UART |

### 3.4 `/metrics_logger` (opsional)

| Parameter | Tipe | Default (kode) | Default (launch) | Keterangan |
|---|---|---|---|---|
| `test_id` | string | `run` | `dist1p5_bright` | ID sesi untuk nama file CSV |
| `distance` | float | `0.0` | `1.5` | Jarak pengujian (meter), dicatat di CSV |
| `light_condition` | string | `unknown` | `bright` | Kondisi cahaya, dicatat di CSV |
| `output_dir` | string | `~/metrics_logs` | `/home/akmal/Documents/finalproject/metrics_logs` | Folder output CSV |
| `flush_interval_sec` | float | `0.5` | (default) | Interval flush file CSV (detik) |
| `latency_total_topic` | string | `/latency/control_serial` | `/latency/control_serial` | Topic latency error-to-serial |
| `latency_control_topic` | string | `/latency/control_compute` | `/latency/control_compute` | Topic latency komputasi PID |
| `latency_serial_topic` | string | `/latency/serial_write` | `/latency/serial_write` | Topic latency penulisan serial |

Catatan: Nama file CSV otomatis dibuat dengan format `run_{test_id}_{timestamp}.csv`. Flush interval 0 atau negatif = flush setiap row (lebih lambat). Subscription ke topic latency bersifat kondisional (hanya jika parameter topic tidak kosong).

## 4) Topic List

Topic utama sistem:

| Topic | Type | Publisher | Subscriber | Fungsi |
|---|---|---|---|---|
| `/vision_error` | `geometry_msgs/msg/Vector3Stamped` | `/camera_vision` | `/pid`, `/metrics_logger` | Error tracking (`vector.x=err_x`, `vector.y=err_y`, `vector.z=confidence`) + timestamp capture |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `/pid` | `/serial`, `/metrics_logger` | Perintah kecepatan pan/tilt |
| `/latency/control_compute` | `std_msgs/msg/Float32` | `/pid` | `/metrics_logger` | Latency komputasi kontrol (ms) |
| `/latency/control_serial` | `std_msgs/msg/Float32` | `/serial` | `/metrics_logger` | Latency error-receive -> serial write (ms) |
| `/latency/serial_write` | `std_msgs/msg/Float32` | `/serial` | `/metrics_logger` | Latency serial write (ms) |

Catatan: tampilan debug hanya berupa window OpenCV lokal.

Topic standar ROS 2 yang juga ada saat node aktif:
- `/parameter_events`
- `/rosout`

## 5) Type dan Interface

### 5.1 Interface yang dipakai runtime

#### `geometry_msgs/msg/Vector3Stamped`
Digunakan pada `/vision_error`.

```text
std_msgs/Header header
geometry_msgs/Vector3 vector
```

Pemaknaan di sistem ini:
- `header.stamp`: timestamp capture frame (untuk vision latency)
- `vector.x`: error horizontal (pixel)
- `vector.y`: error vertikal (pixel)
- `vector.z`: confidence deteksi YOLO (`0.0` jika tidak ada deteksi)
#### `std_msgs/msg/Float32`
Digunakan pada `/latency/control_serial`.

```text
float32 data
```

Pemaknaan di sistem ini:
- `data`: latency (ms) dari error diterima di serial node sampai serial write


#### `geometry_msgs/msg/Twist`
Digunakan pada `/cmd_vel`.

```text
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
```

Pemaknaan di sistem ini:
- `linear.x`: `pan_vel`
- `linear.y`: `tilt_vel`
- Field lain tidak digunakan.

#### Latency topic (opsional)
Tipe pesan latency ditentukan oleh parameter `latency_msg_type` (contoh: `std_msgs/msg/Float32`), dan nilai diambil dari field `latency_field` (default: `data`).

### 5.2 Interface custom dalam paket

File: `msg/ErrorMsg.msg`

```text
int32 err_x
int32 err_y
float32 confidence
```

Status: saat ini belum dipakai oleh node runtime (pipeline aktif memakai `geometry_msgs/msg/Vector3Stamped`).

## 6) Flow Data Sistem

```mermaid
flowchart LR
    CAM[Camera / OpenCV Capture] --> CV[camera_vision node]
    CV -->|/vision_error\nVector3Stamped| PID[pid node]
    PID -->|/cmd_vel\nTwist| SER[serial node]
    SER --> MCU[STM32 via UART]
    CV --> LOG["metrics_logger optional"]
    PID -->|/latency/control_compute\nFloat32| LOG
    SER -->|/latency/control_serial\nFloat32| LOG
    SER -->|/latency/serial_write\nFloat32| LOG
```

Penjelasan alur:
1. `camera_vision` membaca frame, menjalankan YOLO `best.pt` untuk class `Human-body`, lalu menghitung error terhadap pusat frame.
2. Error dipublish ke `/vision_error` sebagai `Vector3Stamped` dengan `header.stamp` saat capture.
3. `pid` menerima error, melakukan pembalikan sumbu pan (`err_x` dibalik), kontrol PID + saturasi + rate limit, lalu publish `/cmd_vel`.
4. `serial` menerima `/cmd_vel` dan mengirim string perintah format `P{pan},T{tilt}\n` ke mikrokontroler.
5. (Opsional) `metrics_logger` mencatat `/vision_error`, `/cmd_vel`, dan latency ke CSV.

## 7) Interface ke Hardware (Serial)

Format command serial dari node:

```text
P+1000,T-500
```

Detail:
- Prefix `P` = pan velocity
- Prefix `T` = tilt velocity
- Nilai bertanda `+/-`
- Diakhiri newline `\n`

## 8) Daftar Dependency Paket

Dari `package.xml`, dependensi utama:
- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `cv_bridge`
- `std_msgs`
- `python3-serial`
- `python3-opencv`
- `ros2launch` (exec depend)

Tambahan Python (pip) untuk node YOLO:
- `ultralytics`
- `numpy`

## 9) Menjalankan Sistem

```bash
cd ros2_ws
colcon build --packages-select pan_tilt_controller --symlink-install
source install/setup.bash
ros2 launch pan_tilt_controller pan_tilt.launch.py
```

Override logger dan window debug:

```bash
ros2 launch pan_tilt_controller pan_tilt.launch.py enable_logger:=false show_debug:=false
```

Pastikan file model `best.pt` tersedia di folder paket.

Cek cepat runtime:

```bash
ros2 node list
ros2 topic list
ros2 topic info /vision_error
ros2 interface show geometry_msgs/msg/Vector3Stamped
```

## 10) Catatan Implementasi Penting

- Node vision memakai model YOLO `best.pt` dengan class target `Human-body` (hardcoded).
- Deadzone terpisah untuk horizontal (`deadzone_h`) dan vertikal (`deadzone_v`) memungkinkan tuning presisi independen.
- Jump filter dengan threshold `max_jump` mencegah deteksi melompat/flicker antar frame.
- Node PID melakukan pembalikan arah pan (`err_x = -msg.vector.x`) agar arah gerak sesuai mekanik.
- Ketika `confidence` = `0.0`, node PID mengirim `pan_vel`/`tilt_vel` = `0.0` dan me-reset semua state PID.
- Ketika target dalam deadzone, integrator direset untuk mencegah windup.
- `show_debug` hanya membuka window OpenCV lokal; tidak ada topic image.
- Anti-windup sederhana diterapkan saat output menyentuh batas saturasi.
- Parameter pada launch override default parameter di kode.
- `accel_limit` mengontrol batas akselerasi velocity (perubahan per detik). Nilai `0` menonaktifkan limit.
- `capture_rate` menentukan frekuensi pembacaan frame (Hz), terpisah dari `inference_rate` YOLO.
- `vertical_ref_ratio` mendefinisikan posisi vertikal reference untuk ROI (default 0.33 = 1/3 dari atas).
- `metrics_logger` menulis CSV ke `output_dir` untuk topic latency yang dikonfigurasi.
- Vision latency dihitung dari `header.stamp` pada `/vision_error` ke waktu publish saat diterima logger.
- Control+serial latency dipublish oleh `serial` ke `/latency/control_serial` (ms).
- Node PID, serial, dan logger memanggil parameter callback untuk hot-reload parameter (belum sepenuhnya terimplementasi untuk semua parameter).


### Alur Data Visualization

![ROS Graph](rosgraph.png)

Diagram ROS node di atas menunjukkan alur komunikasi antar node:
- Camera → camera_vision → pid → serial → MCU
- metrics_logger (opsional) merekam data dari seluruh node

