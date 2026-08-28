import os
import sys
import cv2
import time
import numpy as np
import RPi.GPIO as GPIO

# GPIO Pin setup for servos
xpin = 17
ypin = 27
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(xpin, GPIO.OUT)
GPIO.setup(ypin, GPIO.OUT)
PWM1 = GPIO.PWM(xpin, 50)  # Pan (horizontal)
PWM2 = GPIO.PWM(ypin, 50)  # Tilt (vertical)

# Add src directory to the path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.picamera_utils import is_raspberry_camera, get_picamera

# Camera settings
CAMERA_DEVICE_ID = 0
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
IS_RASPI_CAMERA = is_raspberry_camera()
FPS_TARGET = 60
FRAME_DELAY = 1 / FPS_TARGET  # FPS 30

# HoughCircles Parameters
HOUGH_DP = 1.2        # Inverse ratio of accumulator resolution to image resolution
HOUGH_MIN_DIST = 30   # Minimum distance between circle centers
HOUGH_PARAM1 = 135    # Canny edge detector threshold (upper)
HOUGH_PARAM2 = 55     # Circle detection threshold
HOUGH_MIN_RADIUS = 10  # Minimum circle radius
HOUGH_MAX_RADIUS = 50  # Maximum circle radius

# PID control variables
# Px 기준으로 정규화: 최대 오차(160, 120)일 때 각각 -0.1, -0.02 출력이 나오도록 설계된 값
Px, Ix, Dx = -0.1 / 160, (-0.1 / 160) / 50, (-0.1 / 160) * 0.3
Py, Iy, Dy = -0.02 / 120, (-0.02 / 120) / 50, (-0.02 / 120) * 0.3
integral_x, integral_y = 0, 0
differential_x, differential_y = 0, 0
prev_x, prev_y = 0, 0

# Servo initial positions
currentx, currenty = 5, 11
PWM1.start(currentx)
PWM2.start(currenty)
time.sleep(1)
PWM1.ChangeDutyCycle(0)
PWM2.ChangeDutyCycle(0)


def calculate_pid(error, integral, differential, prev_error, P, I, D, integral_limit=5000):
    """Calculate PID output based on error values."""
    integral += error
    # Anti-windup: 적분값이 한없이 커지는 것을 방지
    integral = max(-integral_limit, min(integral_limit, integral))
    differential = error - prev_error
    pid_output = P * error + I * integral + D * differential
    return round(pid_output, 2), integral, differential


def control_servo1(servo, current_pos, pid_value, axis_label):
    """Control the servo motor smoothly."""
    # Avoid overshooting
    if abs(pid_value) > 0.2:
        step = 0.2 if pid_value > 0 else -0.2
        current_pos += step
    else:
        current_pos += pid_value
    # Clamp duty cycle to avoid invalid values
    current_pos = max(3, min(12, current_pos))
    current_pos = round(current_pos, 2)
    print(f"{axis_label} duty cycle: {current_pos}")
    servo.ChangeDutyCycle(current_pos)
    time.sleep(0.02)
    servo.ChangeDutyCycle(0)
    time.sleep(0.02)
    return current_pos


def control_servo2(servo, current_pos, pid_value, axis_label):
    """Control the servo motor smoothly."""
    # Avoid overshooting
    if abs(pid_value) > 0.2:
        step = 0.2 if pid_value > 0 else -0.2
        current_pos += step
    else:
        current_pos += pid_value
    # Clamp duty cycle to avoid invalid values
    current_pos = max(7, min(12, current_pos))
    current_pos = round(current_pos, 2)
    print(f"{axis_label} duty cycle: {current_pos}")
    servo.ChangeDutyCycle(current_pos)
    time.sleep(0.02)
    servo.ChangeDutyCycle(0)
    return current_pos


def visualize_fps(image, fps):
    """Overlay FPS on the video feed."""
    text_color = (0, 255, 0)
    row_size = 20
    left_margin = 24
    font_size = 1
    font_thickness = 1
    fps_text = f"FPS = {fps:.1f}"
    cv2.putText(image, fps_text, (left_margin, row_size), cv2.FONT_HERSHEY_PLAIN,
                font_size, text_color, font_thickness)
    return image


if __name__ == "__main__":
    try:
        # Camera initialization
        if IS_RASPI_CAMERA:
            print("Using Raspberry Pi Camera")
            cap = get_picamera(IMAGE_WIDTH, IMAGE_HEIGHT)
            cap.start()
        else:
            cap = cv2.VideoCapture(CAMERA_DEVICE_ID)
            cap.set(3, IMAGE_WIDTH)
            cap.set(4, IMAGE_HEIGHT)

        while True:
            start_time = time.time()

            # Capture frame
            if IS_RASPI_CAMERA:
                frame = cap.capture_array()
            else:
                _, frame = cap.read()

            frame = cv2.flip(frame, 1)  # Mirror the frame
            frame = cv2.blur(frame, (3, 3))  # Reduce noise
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            gray = clahe.apply(gray)
            blurred = cv2.blur(gray, (5, 5))

            # Circle detection
            circles = cv2.HoughCircles(
                blurred,
                cv2.HOUGH_GRADIENT,
                dp=HOUGH_DP,
                minDist=HOUGH_MIN_DIST,
                param1=HOUGH_PARAM1,
                param2=HOUGH_PARAM2,
                minRadius=HOUGH_MIN_RADIUS,
                maxRadius=HOUGH_MAX_RADIUS
            )

            PWM1.ChangeDutyCycle(0)
            PWM2.ChangeDutyCycle(0)

            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                for (x, y, r) in circles[:1]:  # Process the first detected circle
                    print(f"Circle detected at: x={x}, y={y}, radius={r}")

                    # Calculate errors
                    error_x = 160 - x
                    error_y = 120 - y

                    # PID calculations
                    valx, integral_x, differential_x = calculate_pid(
                        error_x, integral_x, differential_x, prev_x, Px, Ix, Dx
                    )
                    valy, integral_y, differential_y = calculate_pid(
                        error_y, integral_y, differential_y, prev_y, Py, Iy, Dy
                    )
                    prev_x, prev_y = error_x, error_y

                    # Servo control
                    if abs(error_x) > 20:  # Avoid unnecessary movement
                        currentx = control_servo1(PWM1, currentx, valx, "X")
                    if abs(error_y) > 20:
                        currenty = control_servo2(PWM2, currenty, valy, "Y")

                    # Draw the detected circle
                    frame = cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                    frame = cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

            cv2.imshow("circle", frame)

            # Display results
            elapsed_time = (time.time() - start_time)
            delay = max(0, FRAME_DELAY - elapsed_time)
            time.sleep(delay)

            # Exit condition
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"Error: {e}")

    finally:
        # Cleanup
        cv2.destroyAllWindows()
        cap.close() if IS_RASPI_CAMERA else cap.release()
        PWM1.stop()
        PWM2.stop()
        GPIO.cleanup()
