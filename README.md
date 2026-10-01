# Raspberry Pi Ball Tracking System

임베디드 시스템 설계 기말프로젝트 — Raspberry Pi와 Camera로 공을 검출하고, Pan-Tilt Servo를 PID 제어해 공을 실시간으로 추적하는 시스템입니다.

## 개요

- Raspberry Pi Camera로 영상을 입력받아 OpenCV `HoughCircles`로 공을 검출합니다.
- Blur · Grayscale · CLAHE 전처리를 적용해 영상의 노이즈와 명암 차이를 보정합니다.
- 검출된 공의 중심 좌표와 화면 중심 사이의 X/Y 오차를 계산합니다.
- X/Y 오차를 PID 제어 입력으로 사용해 Pan-Tilt Servo의 PWM Duty Cycle을 조정하고, 공이 화면 중앙을 따라가도록 구현했습니다.
- Servo의 급격한 움직임과 과도한 출력을 방지하기 위해 출력 변화량 제한, Deadband, Duty Cycle Clamp를 적용했습니다.

---

## 하드웨어 구성

<img src="assets/01_hardware.png" width="380">

Raspberry Pi + Camera Module + Pan-Tilt Servo Motor 2개로 구성했습니다.

- Pan Servo: GPIO 17
- Tilt Servo: GPIO 27
- Servo PWM Frequency: 50 Hz

---

## 동작 파이프라인

```text
Camera
→ Blur(3×3)
→ Grayscale
→ CLAHE
→ Blur(5×5)
→ HoughCircles 원 검출
→ 공 중심 좌표 추출
→ 화면 중심과의 X/Y 오차 계산
→ PID 제어
→ 출력 변화량 및 가동 범위 제한
→ PWM Duty Cycle 조정
→ Pan/Tilt Servo 구동
```

---

## 영상 처리 및 원 검출 설정

| 구분 | Parameter | Setting |
|---|---|---|
| Input | Resolution | 320 × 240 |
| Preprocessing | Blur Kernel | 3×3 → 5×5 |
| CLAHE | clipLimit | 2.0 |
| CLAHE | tileGridSize | 8 × 8 |
| HoughCircles | dp | 1.2 |
| HoughCircles | minDist | 30 px |
| HoughCircles | param1 | 135 |
| HoughCircles | param2 | 55 |
| HoughCircles | Radius | 10 ~ 50 px |

입력 영상에 3×3 Blur를 적용해 노이즈를 줄인 뒤 Grayscale로 변환했습니다.  
이후 CLAHE(`clipLimit=2.0`, `tileGridSize=8×8`)를 적용해 국부적인 명암 대비를 보정하고, HoughCircles 입력 전에 5×5 Blur를 추가 적용했습니다.

HoughCircles의 `dp`, `minDist`, Edge Threshold, Circle Detection Threshold 및 검출 반경 범위를 공의 크기에 맞게 설정해 원 검출 조건을 구성했습니다.

---

## 원 검출 (Circle Detection)

CLAHE 전처리와 HoughCircles 파라미터 조정을 통해 공의 원형 경계와 중심 좌표를 검출했습니다.

<img src="assets/02_detection.png" width="380">

검출된 원 중 첫 번째 원의 중심 좌표 `(x, y)`와 반지름 `r`을 이용하고, 화면 중앙 `(160, 120)`과의 오차를 계산했습니다.

```python
error_x = 160 - x
error_y = 120 - y
```

---

## PID 제어

공의 중심과 화면 중심 사이의 X/Y 오차를 기반으로 Pan/Tilt Servo를 PID 제어했습니다.

### PID Gain

| Axis | P | I | D |
|---|---|---|---|
| X (Pan) | `-0.1 / 160` | `Px / 50` | `Px × 0.3` |
| Y (Tilt) | `-0.02 / 120` | `Py / 50` | `Py × 0.3` |

PID 출력은 비례항(P), 적분항(I), 미분항(D)을 모두 반영하도록 구성했습니다.

```python
def calculate_pid(error, integral, differential, prev_error, P, I, D, integral_limit=5000):
    integral += error

    # Anti-windup
    integral = max(-integral_limit, min(integral_limit, integral))

    differential = error - prev_error

    pid_output = (
        P * error
        + I * integral
        + D * differential
    )

    return round(pid_output, 2), integral, differential
```

적분항이 지속적으로 누적되어 제어 출력이 과도하게 증가하는 것을 방지하기 위해 Integral 값을 ±5000 범위로 제한하는 Anti-windup을 적용했습니다.

---

## Servo 출력 안정화

PID 출력이 Servo에 그대로 반영될 경우 큰 오차에서 급격한 움직임이 발생할 수 있어 출력 제한 로직을 추가했습니다.

### 1. Duty Cycle 변화량 제한

한 번의 Servo 제어에서 PID 출력의 절댓값이 0.2보다 큰 경우 Duty Cycle 변화량을 최대 ±0.2로 제한했습니다.

```python
if abs(pid_value) > 0.2:
    step = 0.2 if pid_value > 0 else -0.2
    current_pos += step
else:
    current_pos += pid_value
```

### 2. Servo 가동 범위 제한

Servo의 물리적인 가동 범위를 벗어나지 않도록 Duty Cycle을 Clamp했습니다.

| Servo | Duty Cycle Range |
|---|---|
| X축 (Pan) | 3 ~ 12 |
| Y축 (Tilt) | 7 ~ 12 |

### 3. Deadband 적용

공이 화면 중심 부근에 위치할 때 불필요하게 Servo가 반복 동작하는 것을 줄이기 위해 X/Y 위치 오차가 ±20 px를 초과할 때만 Servo를 구동했습니다.

```python
if abs(error_x) > 20:
    currentx = control_servo1(PWM1, currentx, valx, "X")

if abs(error_y) > 20:
    currenty = control_servo2(PWM2, currenty, valy, "Y")
```

---

## 제어 흐름

```text
공 검출
    ↓
공 중심 좌표 (x, y)
    ↓
화면 중심 (160, 120)과 X/Y 오차 계산
    ↓
PID Control
    ↓
Anti-windup
    ↓
Duty Cycle 변화량 ±0.2 제한
    ↓
Deadband ±20 px
    ↓
Pan/Tilt Duty Cycle Clamp
    ↓
Servo Motor 구동
```

---

## 결과

Raspberry Pi Camera 영상에서 공의 위치를 실시간으로 검출하고, 검출된 공과 화면 중심 사이의 X/Y 오차를 기반으로 Pan-Tilt Servo를 제어했습니다.

PID 제어와 출력 제한 로직을 적용해 공의 위치 변화에 따라 Servo가 움직이며 공을 화면 중심 방향으로 추적하도록 구현했습니다.

- HoughCircles 기반 실시간 공 검출
- X/Y 위치 오차 기반 Pan-Tilt PID 제어
- Integral Anti-windup 적용
- Servo 제어 시 Duty Cycle 변화량 최대 ±0.2 제한
- X축 3-12 / Y축 7-12 Duty Cycle Clamp
- 화면 중심 기준 ±20 px Deadband 적용

---

## 실행 환경

| 항목 | 내용 |
|---|---|
| Board | Raspberry Pi |
| Language | Python 3 |
| Library | OpenCV (`cv2`), `RPi.GPIO`, `NumPy` |
| Camera | Raspberry Pi Camera Module |
| Actuator | Pan-Tilt Servo Motor |
| PWM Frequency | 50 Hz |
| Input Resolution | 320 × 240 |
| Target FPS | 60 |

---

