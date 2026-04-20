#!/usr/bin/env python3
import serial
import sys
import threading
import time
from pynput import keyboard

SERIAL_PORT = "/dev/ttyACM0"  # Ganti sesuai OS
BAUDRATE = 115200
PAN_VEL = 1000
TILT_VEL = 500

ser = None
stop_thread = False

# Track keys currently held to avoid repeat sends
keys_held = set()

def baca_serial():
    while not stop_thread:
        if ser and ser.in_waiting:
            try:
                data = ser.readline().decode(errors='ignore').strip()
                if data:
                    print(f"  [STM32] {data}")
            except Exception:
                pass
        time.sleep(0.01)

def kirim_perintah(cmd):
    if ser and ser.is_open:
        ser.write((cmd + '\n').encode())
        print(f"[KIRIM] {cmd}")

def saat_tekan(key):
    global keys_held
    try:
        if key in keys_held:
            return  # Ignore key repeat
        keys_held.add(key)

        if key == keyboard.Key.right:
            kirim_perintah(f"P+{PAN_VEL}")
        elif key == keyboard.Key.left:
            kirim_perintah(f"P-{PAN_VEL}")
        elif key == keyboard.Key.up:
            kirim_perintah(f"T+{TILT_VEL}")
        elif key == keyboard.Key.down:
            kirim_perintah(f"T-{TILT_VEL}")
        elif hasattr(key, 'char'):
            if key.char == 'q':
                return False
            elif key.char == 's':
                kirim_perintah("S")
            elif key.char == '?':
                kirim_perintah("?")
            elif key.char == '1':
                kirim_perintah("IP")  # Toggle pan direction
            elif key.char == '2':
                kirim_perintah("IT")  # Toggle tilt direction
    except AttributeError:
        pass

def saat_lepas(key):
    global keys_held
    try:
        keys_held.discard(key)

        if key in (keyboard.Key.right, keyboard.Key.left):
            kirim_perintah("P+0")
        elif key in (keyboard.Key.up, keyboard.Key.down):
            kirim_perintah("T+0")
    except AttributeError:
        pass

def main():
    global ser, stop_thread
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        print(f"Terhubung ke {SERIAL_PORT}")
    except Exception as e:
        print(f"Gagal: {e}")
        sys.exit(1)

    # Thread pembaca
    thread = threading.Thread(target=baca_serial, daemon=True)
    thread.start()

    print("=== Pan-Tilt Controller ===")
    print("Arrow keys : Gerakkan pan/tilt")
    print("s          : Stop semua")
    print("1          : Toggle invert arah pan")
    print("2          : Toggle invert arah tilt")
    print("?          : Query status")
    print("q          : Keluar")
    print("===========================")

    with keyboard.Listener(on_press=saat_tekan, on_release=saat_lepas) as listener:
        listener.join()

    stop_thread = True
    thread.join(timeout=1)
    ser.close()
    print("Selesai.")

if __name__ == "__main__":
    main()