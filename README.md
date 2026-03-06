# Drone Competition - Hướng dẫn đầy đủ

Dự án điều khiển drone tự động bằng C/C++, chạy trên Raspberry Pi 5.
Phát hiện vòng bay (hoop) và điểm đáp (landing pad) bằng HSV màu sắc hoặc AI YOLOv8.

---

## Mục lục

1. [Cấu trúc project](#1-cấu-trúc-project)
2. [Từng file làm gì](#2-từng-file-làm-gì)
3. [Cài đặt thư viện](#3-cài-đặt-thư-viện)
4. [Cách build](#4-cách-build)
5. [Chạy từng chế độ chi tiết](#5-chạy-từng-chế-độ-chi-tiết)
6. [Thuật toán HSV hoạt động thế nào](#6-thuật-toán-hsv-hoạt-động-thế-nào)
7. [Luồng chạy toàn hệ thống](#7-luồng-chạy-toàn-hệ-thống)
8. [Cấu hình HSV - chỉnh ngưỡng màu](#8-cấu-hình-hsv---chỉnh-ngưỡng-màu)
9. [Lỗi thường gặp và cách sửa](#9-lỗi-thường-gặp-và-cách-sửa)
10. [So sánh hiệu năng Python vs C++](#10-so-sánh-hiệu-năng-python-vs-c)

---

## 1. Cấu trúc project

```
drone_c/
│
├── src/                     <- Code nguồn chính
│   ├── main.cpp             <- Điểm vào chương trình, quản lý toàn bộ
│   ├── hsv_detector.cpp     <- Thuật toán detect vòng bằng màu HSV
│   ├── detector.cpp         <- Detect bằng AI YOLOv8 (NCNN)
│   ├── sensor_fusion.cpp    <- Tổng hợp UWB + camera + IMU → vị trí chính xác
│   ├── pid.c                <- Bộ điều khiển PID
│   ├── kalman2d.c           <- Bộ lọc Kalman 2D cho UWB
│   ├── visual_servo.c       <- Tính lệnh bay để căn vào tâm vòng
│   ├── mission.c            <- Quản lý nhiệm vụ (state machine)
│   ├── flight_controller.c  <- Giao tiếp MAVLink với Pixhawk
│   ├── dwm1001.c            <- Đọc cảm biến UWB DWM1001
│   └── ball_dropper.c       <- Điều khiển servo thả bóng
│
├── include/                 <- Header files (định nghĩa struct, API)
│   ├── detector.h           <- Struct Det (kết quả detect 1 vật thể)
│   ├── hsv_detector.h       <- HSVConfig (ngưỡng màu)
│   ├── visual_servo.h       <- VSResult, vs_compute()
│   ├── mission.h            <- Waypoint, MissionCmd, state machine
│   ├── sensor_fusion.h      <- SensorFusion, FusedState
│   ├── flight_controller.h  <- FC, fc_connect(), fc_vel()
│   ├── pid.h                <- PIDCtrl, pid_update()
│   ├── kalman2d.h           <- Kalman2D
│   ├── dwm1001.h            <- UWBNav
│   └── ball_dropper.h       <- BallDropper, bd_drop_and_close()
│
├── config/
│   └── config.yaml          <- Cấu hình camera, UWB, PID, mission
│
├── models/                  <- File model AI (cần download riêng)
│   ├── yolov8n.ncnn.param   <- Cấu trúc mạng
│   └── yolov8n.ncnn.bin     <- Trọng số mạng
│
├── train/
│   └── train_model.py       <- Script train model YOLOv8 bằng Python
│
├── deps/
│   └── c_library_v2/        <- MAVLink C headers (git submodule)
│
├── build/                   <- Thư mục build (tự tạo)
│   └── drone_competition    <- File thực thi sau khi build
│
└── CMakeLists.txt           <- Cấu hình build
```

---

## 2. Từng file làm gì

### `src/main.cpp` - Trung tâm điều phối
- Parse argument dòng lệnh (`--test-hsv`, `--video`, `--sim`, ...)
- Mở camera hoặc video file
- Khởi động tất cả module
- Vòng lặp chính: đọc frame → detect → tính lệnh bay → gửi Pixhawk

### `src/hsv_detector.cpp` - Detect vòng bằng màu
- Nhận vào 1 frame BGR từ camera/video
- Chuyển sang không gian màu HSV
- Lọc từng màu: đỏ, vàng, xanh, trắng
- Tìm contour (đường viền) của vùng màu
- Tính tâm (centroid) của mỗi contour
- Vẽ kết quả lên frame nếu cần
- Trả về danh sách `Det[]` (tối đa 20 vật thể)

### `src/detector.cpp` - Detect bằng AI YOLOv8
- Load model NCNN từ file `.param` + `.bin`
- Resize + letterbox frame về 320x320
- Chạy inference trên CPU
- NMS (Non-Maximum Suppression) lọc box trùng
- Chạy bất đồng bộ trên thread riêng để không block camera

### `src/sensor_fusion.cpp` - EKF tổng hợp cảm biến
- Kết hợp: UWB (vị trí thô) + Optical Flow (vận tốc camera) + IMU (gia tốc) + Rangefinder (độ cao)
- Dùng Extended Kalman Filter → vị trí chính xác hơn từng cảm biến đơn lẻ

### `src/visual_servo.c` - Điều khiển bám vòng
- Nhận tâm vòng detect được
- Tính sai lệch với tâm frame
- Chạy PID → ra lệnh bay `vx, vy` để drone tự căn vào tâm vòng

### `src/mission.c` - Trình tự nhiệm vụ
- State machine: `NAVIGATE → ALIGN → FLY_THROUGH → DROP_BALL → LAND → DONE`
- Quản lý danh sách waypoint từ `config.yaml`
- Quyết định khi nào chuyển sang waypoint tiếp theo

### `src/flight_controller.c` - Giao tiếp Pixhawk
- Kết nối MAVLink qua UDP (ip:14550)
- Arm drone, takeoff, gửi lệnh vận tốc, hạ cánh
- Nhận dữ liệu IMU, barometer, rangefinder từ Pixhawk

### `src/dwm1001.c` - Cảm biến UWB
- Đọc tọa độ X,Y thô từ module DWM1001 qua cổng serial
- Lọc Kalman 2D → tọa độ mượt hơn
- Chạy trên thread riêng, tốc độ 30 Hz

### `src/pid.c` - Bộ điều khiển PID
- PID cơ bản với anti-windup
- Dùng cho visual servo (căn tâm vòng) và altitude control

### `src/kalman2d.c` - Bộ lọc Kalman
- Lọc nhiễu tọa độ UWB theo 2 trục X,Y
- Dự đoán vị trí khi mất tín hiệu UWB

### `src/ball_dropper.c` - Thả bóng
- Điều khiển servo qua PWM chip trên RPi5
- Mở servo 500ms rồi đóng lại

---

## 3. Cài đặt thư viện

### Bắt buộc (để chạy HSV detect):
```bash
sudo apt update
sudo apt install -y libopencv-dev cmake build-essential
```

### Tùy chọn (để chạy AI YOLOv8):
```bash
# Build NCNN từ source
git clone https://github.com/Tencent/ncnn.git
cd ncnn
mkdir build && cd build
cmake -DNCNN_VULKAN=OFF -DNCNN_BUILD_EXAMPLES=OFF ..
make -j4
sudo make install
cd ../..
```

### MAVLink headers (để chạy full system):
```bash
cd deps
git clone https://github.com/mavlink/c_library_v2.git
```

### Kiểm tra đã cài đúng chưa:
```bash
# Kiểm tra OpenCV
pkg-config --modversion opencv4
# Output ví dụ: 4.5.4

# Kiểm tra NCNN (nếu cài)
ls /usr/local/lib/libncnn.a
# Output: /usr/local/lib/libncnn.a
```

---

## 4. Cách build

```bash
# Bước 1: Vào thư mục project
cd /home/nolan/Documents/drone_c

# Bước 2: Tạo thư mục build
mkdir -p build
cd build

# Bước 3: Configure
cmake .. -DCMAKE_BUILD_TYPE=Release

# Bước 4: Build (j4 = dùng 4 CPU core)
make -j4
```

**Build thành công sẽ thấy:**
```
[100%] Linking CXX executable drone_competition
[100%] Built target drone_competition
```

**File thực thi:** `build/drone_competition`

> **Mỗi lần sửa code** phải chạy lại `make -j4` trong thư mục `build/`.
> Không cần chạy `cmake ..` lại trừ khi sửa `CMakeLists.txt`.

---

## 5. Chạy từng chế độ chi tiết

### 5.1 Test HSV với video (quan trọng nhất)

**Mục đích:** Kiểm tra thuật toán HSV có detect đúng vòng và điểm đáp trong video của bạn không.

**Yêu cầu:** File video `.mp4` / `.avi` / `.mov`

```bash
cd /home/nolan/Documents/drone_c/build

./drone_competition --test-hsv --video ../tên_video.mp4
```

**Ví dụ cụ thể:**
```bash
# Nếu video nằm trong thư mục project
./drone_competition --test-hsv --video ../video_test.mp4

# Nếu video ở Desktop
./drone_competition --test-hsv --video /home/nolan/Desktop/test.mp4

# Nếu video ở thư mục hiện tại
./drone_competition --test-hsv --video ./recording.avi
```

**Output terminal (in mỗi frame):**
```
[TEST]HSV detection mode
[VIDEO]Opened ../video.mp4  640x480@30
[TEST]Press 'q' to quit, SPACE to pause
HSV: 2 detections
  [hoop_red]    cx=320 cy=240 area=0.045
  [white_sq]    cx=150 cy=300 area=0.020
HSV: 1 detections
  [hoop_yellow] cx=410 cy=200 area=0.032
HSV: 0 detections
...
```

**Giải thích output:**
- `cx, cy` = tọa độ tâm vật thể detect được (pixel)
- `area` = tỉ lệ diện tích so với toàn frame (0.045 = 4.5% diện tích frame)
- `hoop_red` / `hoop_yellow` / `hoop_blue` = vòng bay theo màu
- `white_sq` = điểm đáp hình vuông trắng

**Cửa sổ hiển thị "HSV Detection":**

| Màu vẽ | Ý nghĩa |
|--------|---------|
| Đường xanh lá dày | Contour bao quanh vòng detect |
| Vòng tròn xanh lá nhỏ | Safe zone = 2/3 bán kính vòng (vùng an toàn để bay qua) |
| Chấm đỏ | Tâm vòng (centroid) |
| Đường vàng | Vector từ tâm frame đến tâm vòng (hướng drone cần bay) |
| Chữ vàng `vx:.. vy:..` | Lệnh vận tốc tính toán để căn vào tâm |
| Chấm xanh lá ở giữa frame | Tâm frame = điểm chuẩn (drone cần đưa vòng về đây) |
| Đường trắng | Contour điểm đáp trắng |
| Chữ WHITE SQ | Nhãn điểm đáp |

**Phím điều khiển:**
- `SPACE` → Dừng / Tiếp tục video
- `q` hoặc `ESC` → Thoát
- Hết video → Tự động restart từ đầu

---

### 5.2 Test HSV với camera thật

**Mục đích:** Chạy HSV detect real-time từ camera USB/built-in.

```bash
./drone_competition --test-hsv
```

Hoạt động giống hệt 5.1 nhưng đọc từ `/dev/video0` (camera index 0).

**Kiểm tra camera đang ở index nào:**
```bash
ls /dev/video*
# Output ví dụ: /dev/video0  /dev/video2
```

---

### 5.3 Test camera đơn giản (không detect)

**Mục đích:** Kiểm tra camera hoặc video có mở được không, trước khi chạy detect.

```bash
# Với camera
./drone_competition --test-camera

# Với video
./drone_competition --test-camera --video ../video.mp4
```

**Output:**
```
[TEST]Camera mode
Frame 640x480
Frame 640x480
Frame 640x480
...
```

Nếu thấy in `Frame 640x480` liên tục là camera/video hoạt động tốt.
Nếu không in gì hoặc báo lỗi → xem phần Lỗi thường gặp.

---

### 5.4 Test UWB định vị

**Mục đích:** Kiểm tra cảm biến UWB DWM1001 đọc đúng tọa độ không.

**Yêu cầu:** Cắm thiết bị UWB vào cổng USB của RPi5.

```bash
./drone_competition --test-uwb
```

**Output:**
```
[TEST]UWB mode
Raw:1200,800,0  Filt:1195,802  Q:3
Raw:1205,798,0  Filt:1197,801  Q:3
Raw:1210,795,0  Filt:1200,799  Q:2
```

**Giải thích:**
- `Raw:X,Y,Z` = tọa độ thô (mm) từ UWB chưa lọc
- `Filt:X,Y` = tọa độ sau bộ lọc Kalman (mượt hơn)
- `Q:3` = chất lượng tín hiệu (0=xấu, 3=tốt nhất)

---

### 5.5 Chạy full system - Simulation (không cần Pixhawk)

**Mục đích:** Test toàn bộ logic mission, visual servo, fusion... mà không cần drone thật.

```bash
# Dùng HSV detect + simulation
./drone_competition --hsv --sim

# Dùng HSV detect + video test + simulation
./drone_competition --hsv --sim --video ../video.mp4
```

**Chế độ `--sim`:** Bỏ qua kết nối Pixhawk, in lệnh bay thay vì gửi thật.

**Output (mỗi 30 frame):**
```
--- Frame 30 ---
  FPS:28  Det:1  State:1  WP:0
  Pos:(1200,800)  Q:3  Alt:1.50
  Fusion: xy_unc=12.3 z_unc=0.05 UWB:30 Flow:28 IMU:150 Range:30
  Vel:(0.12, -0.05, 0.00)  Bias:(0.001, 0.002)
  Vel:0.12,-0.05,0.00
  [hoop_red] c=1.00 cx=310 cy=235
```

**Giải thích:**
- `State:1` = state machine đang ở trạng thái nào (1=NAVIGATE, 2=ALIGN, 3=FLY_THROUGH, ...)
- `WP:0` = đang đến waypoint số 0
- `Fusion: xy_unc` = độ không chắc chắn vị trí (mm), càng nhỏ càng tốt
- `UWB:30 Flow:28` = số lần update từ UWB và optical flow

---

### 5.6 Chạy full system thật với Pixhawk

**Yêu cầu:** Pixhawk kết nối qua UDP `127.0.0.1:14550`, UWB cắm vào `/dev/ttyACM0`, camera hoạt động.

```bash
# Dùng HSV detect
./drone_competition --hsv

# Dùng AI detect
./drone_competition --ai
```

> **CẢNH BÁO:** Chế độ này sẽ arm drone và takeoff thật. Chỉ chạy khi đã test xong tất cả các chế độ khác.

---

## 6. Thuật toán HSV hoạt động thế nào

File: [src/hsv_detector.cpp](src/hsv_detector.cpp)

### Bước 1: Chuyển màu
```
Frame BGR  →  cv::cvtColor  →  Frame HSV
```
HSV (Hue-Saturation-Value) tách biệt màu sắc (H) ra khỏi độ sáng (V), giúp detect màu ổn định hơn khi ánh sáng thay đổi.

### Bước 2: Lọc màu (inRange)
Tạo mask nhị phân cho từng màu:
```
Mask đỏ:    H: 0-10,   S: 100-255, V: 100-255
Mask vàng:  H: 20-30,  S: 100-255, V: 100-255
Mask xanh:  H: 100-130, S: 100-255, V: 100-255
Mask trắng: H: 0-180,  S: 0-40,   V: 200-255
```
Pixel nào nằm trong range → trắng (255), ngoài → đen (0).

### Bước 3: Tìm contour
```
findContours(mask) → danh sách đường viền
Lọc: diện tích < 500 pixel → bỏ qua (loại nhiễu)
```

### Bước 4: Tính tâm và phân loại
```
moments(contour) → m00, m10, m01
cx = m10 / m00
cy = m01 / m00
Kiểm tra pixel (cx,cy) nằm trong mask nào → đặt tên hoop_red/yellow/blue
```

### Bước 5: Detect điểm đáp trắng
```
findContours(mask_trắng)
approxPolyDP → xấp xỉ đa giác
Điều kiện: 4 đỉnh + tỉ lệ width/height: 0.8-1.2 → là hình vuông
```

### Bước 6: Vẽ kết quả
```
drawContours → đường viền xanh lá
minEnclosingCircle → vòng safe zone (2/3 radius)
circle tâm đỏ
line từ tâm frame → tâm vòng (vector hướng bay)
```

---

## 7. Luồng chạy toàn hệ thống

```
video.mp4 hoặc camera
       |
       v
  cap.read(frame)          [main.cpp]
       |
       +---> hsv_detect()  [hsv_detector.cpp]
       |     - BGR -> HSV
       |     - inRange -> mask
       |     - findContours
       |     - moments -> (cx, cy)
       |     - trả về Det[]
       |
       +---> oft_process() [sensor_fusion.cpp]
       |     - Optical flow từ frame
       |     - sf_update_optflow()
       |
       +---> uwb_filtered() [dwm1001.c - thread riêng]
       |     - Đọc UWB
       |     - sf_update_uwb()
       |
       v
  sf_get_position()        [sensor_fusion.cpp]
  -> pos_xy (vị trí fused)
       |
       v
  vs_compute(dets, pos)    [visual_servo.c]
  -> vsr.vx, vsr.vy, vsr.vz
  -> vsr.stable, vsr.passed
       |
       v
  mission_update(pos, vsr) [mission.c]
  -> mc.vx, mc.vy, mc.vz
  -> mc.do_drop, mc.alt
       |
       v
  fc_vel(vx, vy, vz)       [flight_controller.c]
  -> MAVLink SET_POSITION_TARGET
  -> Pixhawk6C -> Motor
```

---

## 8. Cấu hình HSV - chỉnh ngưỡng màu

File cấu hình ngưỡng: [src/hsv_detector.cpp:13-19](src/hsv_detector.cpp#L13-L19)

Nếu detect sai màu hoặc bỏ sót, chỉnh các giá trị này trong `hsv_config_default()`:

```c
// Màu đỏ: H từ 0 đến 10
c->red_h_lo = 0;    c->red_h_hi = 10;
c->red_s_lo = 100;  c->red_v_lo = 100;

// Màu vàng: H từ 20 đến 30
c->yellow_h_lo = 20; c->yellow_h_hi = 30;
c->yellow_s_lo = 100; c->yellow_v_lo = 100;

// Màu xanh lam: H từ 100 đến 130
c->blue_h_lo = 100;  c->blue_h_hi = 130;
c->blue_s_lo = 100;  c->blue_v_lo = 100;

// Trắng: S thấp (< 40), V cao (> 200)
c->white_s_hi = 40;  c->white_v_lo = 200;

// Diện tích tối thiểu (pixel^2), nhỏ hơn thì bỏ qua
c->min_area = 500;
```

**Bảng giá trị H trong OpenCV (0-180):**

| Màu | H (OpenCV) |
|-----|-----------|
| Đỏ | 0-10 và 170-180 |
| Cam | 10-20 |
| Vàng | 20-30 |
| Xanh lá | 40-80 |
| Xanh lam | 100-130 |
| Tím | 130-160 |

> Lưu ý: OpenCV dùng H từ 0-180 (không phải 0-360 như thông thường).

**Cách chỉnh khi detect sai:**
- Detect quá nhiều nhiễu → tăng `min_area`, tăng `s_lo` và `v_lo`
- Bỏ sót vòng → mở rộng range H (giảm `h_lo`, tăng `h_hi`)
- Nhạy với ánh sáng → giảm `v_lo` và `s_lo`

Sau khi sửa phải **build lại:**
```bash
cd build && make -j4
```

---

## 9. Lỗi thường gặp và cách sửa

### Lỗi: `Failed open video: ../video.mp4`
```
Nguyên nhân: Đường dẫn video sai hoặc file không tồn tại.
Sửa:
  ls ../           # xem file nào đang có
  # Dùng đường dẫn tuyệt đối:
  ./drone_competition --test-hsv --video /home/nolan/Documents/drone_c/video.mp4
```

### Lỗi: `Failed open 0` (mở camera thất bại)
```
Nguyên nhân: Không tìm thấy camera ở index 0.
Sửa:
  ls /dev/video*   # xem camera ở index nào
  # Nếu ở /dev/video2 thì camera index là 2
  # Hiện tại chưa có argument --cam-idx, dùng video để test thay thế
```

### Lỗi build: `opencv2/core.hpp: No such file or directory`
```
Sửa:
  sudo apt install libopencv-dev
```

### Lỗi build: `ncnn/net.h: No such file or directory`
```
Sửa (cách 1): Build và install NCNN như hướng dẫn phần 3
Sửa (cách 2): Chỉ chạy HSV mode, không cần NCNN
  # NCNN chỉ cần khi dùng --ai flag
```

### Lỗi build: `cannot find -lncnn`
```
Sửa:
  sudo ldconfig   # refresh thư viện
  # Hoặc chỉ định path:
  cmake .. -DNCNN_LIB=/usr/local/lib/libncnn.a
```

### Cửa sổ không hiện lên khi chạy --test-hsv
```
Nguyên nhân: Môi trường không có display (SSH không có X11 forwarding).
Sửa (cách 1): Thêm X11 forwarding khi SSH:
  ssh -X user@raspberrypi
Sửa (cách 2): Chạy trực tiếp trên màn hình RPi5
Sửa (cách 3): Chỉ xem output terminal, không cần cửa sổ
```

### UWB báo lỗi mở port
```
Nguyên nhân: Thiếu quyền hoặc không có thiết bị.
Sửa:
  sudo chmod 666 /dev/ttyACM0
  # Hoặc thêm user vào group dialout:
  sudo usermod -a -G dialout $USER
  # Rồi logout và login lại
```

---

## 10. So sánh hiệu năng Python vs C++

| Module | Python | C++ | Tăng tốc |
|--------|--------|-----|---------|
| YOLO inference (320x320) | ~10 FPS | ~50 FPS | 5x |
| HSV detection (640x480) | ~30 FPS | ~100+ FPS | 3x+ |
| Control loop | ~20 Hz | ~50 Hz | 2.5x |
| UWB polling | 30 Hz | 30 Hz | tương đương |
| Latency tổng | ~50ms | ~15ms | 3x |
| RAM sử dụng | ~200MB | ~50MB | 4x ít hơn |

---

## Tóm tắt nhanh - Bảng lệnh

| Mục tiêu | Lệnh (chạy trong thư mục `build/`) |
|----------|-------------------------------------|
| Build code | `cd build && make -j4` |
| **Test HSV + video** | `./drone_competition --test-hsv --video ../video.mp4` |
| Test HSV + camera | `./drone_competition --test-hsv` |
| Kiểm tra video mở được không | `./drone_competition --test-camera --video ../video.mp4` |
| Kiểm tra camera | `./drone_competition --test-camera` |
| Test UWB (cần cắm thiết bị) | `./drone_competition --test-uwb` |
| Chạy full, không có Pixhawk | `./drone_competition --sim --hsv` |
| Chạy full, không có Pixhawk + video | `./drone_competition --sim --hsv --video ../video.mp4` |
| Chạy thật (HSV) | `./drone_competition --hsv` |
| Chạy thật (AI) | `./drone_competition --ai` |
