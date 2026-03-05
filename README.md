# Drone Competition - C/C++ Version

## Build trn RPi5

```bash
# 1. Install dependencies
sudo apt install -y libopencv-dev cmake build-essential

# 2. Build NCNN (optional, for AI detection)
git clone https://github.com/Tencent/ncnn.git
cd ncnn && mkdir build && cd build
cmake -DNCNN_VULKAN=OFF -DNCNN_BUILD_EXAMPLES=OFF ..
make -j4 && sudo make install

# 3. Get MAVLink headers
cd deps && git clone https://github.com/mavlink/c_library_v2.git

# 4. Build project
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
```

## Run

```bash
# Test HSV detection (no AI needed)
./drone_competition --test-hsv

# Test HSV detection with a recorded video file
./drone_competition --test-hsv --video /path/to/test.mp4

# Test camera
./drone_competition --test-camera

# Test UWB
./drone_competition --test-uwb

# Full mission with HSV detection
./drone_competition --hsv

# Full mission with AI (NCNN YOLO)
./drone_competition --ai

# Simulation mode (no FC connection)
./drone_competition --hsv --sim
```

## Project Structure

```
include/          - Header files
  pid.h           - PID controller
  kalman2d.h      - Kalman filter 2D
  dwm1001.h       - UWB DWM1001 driver
  detector.h      - NCNN YOLO detector
  hsv_detector.h  - HSV color detection (port of process.py)
  visual_servo.h  - Visual servoing
  mission.h       - Mission state machine
  flight_controller.h - MAVLink
  ball_dropper.h  - Servo PWM

src/              - Source files
  pid.c           - PID (port of pid.py)
  kalman2d.c      - Kalman (port of uwb_nav.py KalmanFilter2D)
  dwm1001.c       - UWB (port of decawave_1001.py + uwb_nav.py)
  detector.cpp    - NCNN YOLO (port of detector.py)
  hsv_detector.cpp - HSV detection (port of process.py + input.py)
  visual_servo.c  - Visual servo (port of visual_servo.py)
  mission.c       - Mission FSM (port of mission.py)
  flight_controller.c - MAVLink (port of flight_controller.py)
  ball_dropper.c  - Servo (port of ball_dropper.py)
  main.cpp        - Main loop (port of main.py)

config/           - Configuration
models/           - NCNN model files
deps/             - MAVLink C headers
train/            - Training script (Python)
```

## Detection Modes

### HSV Color (--hsv)
- Port of process.py: detect red/yellow/blue hoops + white squares
- No AI model needed, works immediately
- Good for testing and simple environments

### AI NCNN YOLO (--ai)
- Port of detector.py: YOLOv8n NCNN inference
- Need trained model (.param + .bin)
- Better accuracy, handles complex scenes
- 15-20 FPS on RPi5 (320x320 input)

## Performance vs Python

| Module | Python | C/C++ | Speedup |
|--------|--------|-------|---------|
| YOLO inference | ~10 FPS | ~50 FPS (320) | 5x |
| HSV detection | ~30 FPS | ~100+ FPS | 3x |
| Control loop | ~20 Hz | ~50 Hz | 2.5x |
| UWB polling | 30 Hz | 30 Hz | same |
| Total latency | ~50ms | ~15ms | 3x |
# Drone_competition
