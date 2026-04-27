import threading
import time

import cv2
import numpy as np
import serial
from pynput import keyboard

SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200

PAN_SPEED_MANUAL = 1000
TILT_SPEED_MANUAL = 500
PAN_DIRECTION = -1

GAIN = 6
MAX_PAN = 1500
MAX_TILT = 1000
DEADZONE_X = 40
DEADZONE_Y = 30

LOW_GREEN = np.array([35, 80, 80])
HIGH_GREEN = np.array([85, 255, 255])
MIN_CONTOUR_AREA = 500
CAMERA_INDEX = 0

ser = None
listener_ref = None
stop_event = threading.Event()
state_lock = threading.Lock()

auto_mode = True
keys_held = set()
last_cmd = {"P": None, "T": None}


def clamp(value, low, high):
    return max(low, min(high, value))


def set_auto_mode(enabled):
    global auto_mode
    with state_lock:
        auto_mode = enabled
    print(f"[MODE] {'AUTO' if enabled else 'MANUAL'}")


def get_auto_mode():
    with state_lock:
        return auto_mode


def kirim(cmd):
    if ser and ser.is_open:
        ser.write((cmd + "\n").encode())
        print(f"[KIRIM] {cmd}")


def set_axis_velocity(axis, velocity):
    velocity = int(velocity)

    if axis == "P":
        velocity *= PAN_DIRECTION
    elif axis != "T":
        return

    with state_lock:
        if last_cmd[axis] == velocity:
            return
        last_cmd[axis] = velocity

    kirim(f"{axis}{velocity:+d}")


def stop_motion():
    set_axis_velocity("P", 0)
    set_axis_velocity("T", 0)


def compute_velocity(error, deadzone, gain, vmax):
    if abs(error) <= deadzone:
        return 0
    return clamp(int(gain * error), -vmax, vmax)


def toggle_mode():
    set_auto_mode(not get_auto_mode())
    stop_motion()


def baca_serial():
    while not stop_event.is_set():
        if ser and ser.is_open and ser.in_waiting:
            try:
                msg = ser.readline().decode(errors="ignore").strip()
                if msg:
                    print(f"  [STM32] {msg}")
            except Exception:
                pass
        time.sleep(0.01)


def saat_tekan(key):
    try:
        if key in keys_held:
            return
        keys_held.add(key)

        if key == keyboard.Key.right:
            set_auto_mode(False)
            set_axis_velocity("P", PAN_SPEED_MANUAL)
        elif key == keyboard.Key.left:
            set_auto_mode(False)
            set_axis_velocity("P", -PAN_SPEED_MANUAL)
        elif key == keyboard.Key.up:
            set_auto_mode(False)
            set_axis_velocity("T", TILT_SPEED_MANUAL)
        elif key == keyboard.Key.down:
            set_auto_mode(False)
            set_axis_velocity("T", -TILT_SPEED_MANUAL)
        elif hasattr(key, "char") and key.char:
            c = key.char.lower()
            if c == "m":
                toggle_mode()
            elif c == "s":
                stop_motion()
            elif c == "q":
                stop_event.set()
                return False
    except Exception:
        pass


def saat_lepas(key):
    try:
        keys_held.discard(key)

        if key in (keyboard.Key.right, keyboard.Key.left):
            set_axis_velocity("P", 0)
        elif key in (keyboard.Key.up, keyboard.Key.down):
            set_axis_velocity("T", 0)
    except Exception:
        pass


def detect_green_target(frame):
    h, w = frame.shape[:2]
    cx_frame, cy_frame = w // 2, h // 2

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOW_GREEN, HIGH_GREEN)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False, 0, 0, None

    biggest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(biggest) <= MIN_CONTOUR_AREA:
        return False, 0, 0, None

    x, y, bw, bh = cv2.boundingRect(biggest)
    cx = x + bw // 2
    cy = y + bh // 2
    err_x = cx - cx_frame
    err_y = cy_frame - cy
    return True, err_x, err_y, (x, y, bw, bh, cx, cy)


def open_camera_simple():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if cap.isOpened():
        return cap, f"index {CAMERA_INDEX}"
    cap.release()

    if CAMERA_INDEX != 0:
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            return cap, "index 0"
        cap.release()

    return None, None


def run_camera_loop():
    cap, cam_label = open_camera_simple()
    if cap is None:
        print("Kamera tidak ditemukan. Mode manual keyboard tetap aktif.")
        return False

    print(f"Kamera aktif: {cam_label}")

    while not stop_event.is_set():
        ok, frame = cap.read()
        if not ok:
            break

        frame = cv2.flip(frame, 1)

        h, w = frame.shape[:2]
        cx_frame, cy_frame = w // 2, h // 2
        zx = cx_frame - w // 4
        zy = cy_frame - h // 4
        zw = w // 2
        zh = h // 2

        status = "NO TARGET"
        status_color = (0, 0, 255)

        found, err_x, err_y, target = detect_green_target(frame)
        if found and target is not None:
            x, y, bw, bh, cx, cy = target
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(
                frame,
                f"err:(PAN:{err_x:+d},TILT:{err_y:+d})",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            if zx <= cx <= zx + zw and zy <= cy <= zy + zh:
                status = "STEADY"
                status_color = (0, 255, 0)
            else:
                status = "MOVING"
                status_color = (0, 165, 255)

        if get_auto_mode():
            if found:
                pan = compute_velocity(err_x, DEADZONE_X, GAIN, MAX_PAN)
                tilt = compute_velocity(err_y, DEADZONE_Y, GAIN, MAX_TILT)
                set_axis_velocity("P", pan)
                set_axis_velocity("T", tilt)
            else:
                stop_motion()

        mode_text = "AUTO" if get_auto_mode() else "MANUAL"

        cv2.rectangle(frame, (zx, zy), (zx + zw, zy + zh), (255, 255, 0), 1)
        cv2.putText(frame, status, (zx, zy - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.putText(
            frame,
            f"MODE: {mode_text} (m=toggle, s=stop, q=keluar)",
            (10, h - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
        cv2.drawMarker(frame, (cx_frame, cy_frame), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)

        cv2.imshow("Green Tracking + Serial Control", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            stop_event.set()
            break
        if key == ord("m"):
            toggle_mode()
        if key == ord("s"):
            stop_motion()

    cap.release()
    cv2.destroyAllWindows()
    return True


def main():
    global ser, listener_ref

    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        print(f"Terhubung ke {SERIAL_PORT}")
    except Exception as e:
        print(f"Gagal koneksi serial: {e}")
        return

    serial_thread = threading.Thread(target=baca_serial, daemon=True)
    serial_thread.start()

    listener_ref = keyboard.Listener(on_press=saat_tekan, on_release=saat_lepas)
    listener_ref.start()

    print("=== Move Test Sederhana ===")
    print("Arrow keys : Manual pan/tilt")
    print("m          : Toggle AUTO/MANUAL")
    print("s          : Stop")
    print("q          : Keluar")
    print("===========================")

    camera_ok = run_camera_loop()

    if not camera_ok:
        set_auto_mode(False)
        print("Tanpa kamera. Gunakan keyboard, tekan q untuk keluar.")
        try:
            while not stop_event.is_set():
                time.sleep(0.1)
        except KeyboardInterrupt:
            stop_event.set()

    stop_event.set()
    stop_motion()
    kirim("S")

    if listener_ref is not None:
        listener_ref.stop()
        listener_ref.join(timeout=1)

    serial_thread.join(timeout=1)

    if ser and ser.is_open:
        ser.close()

    print("Selesai.")


if __name__ == "__main__":
    main()