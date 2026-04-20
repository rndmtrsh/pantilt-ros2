#!/usr/bin/env python3
import glob
import re
import threading
import time

import cv2
import numpy as np
import serial
from pynput import keyboard

SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200

PAN_VEL_MANUAL = 1000
TILT_VEL_MANUAL = 500

AUTO_GAIN = 6
AUTO_MAX_PAN = 1200
AUTO_MAX_TILT = 800
DEADZONE_X = 40
DEADZONE_Y = 30

LOW_GREEN = np.array([35, 80, 80])
HIGH_GREEN = np.array([85, 255, 255])
MIN_CONTOUR_AREA = 500

ser = None
stop_event = threading.Event()
keys_held = set()
listener_ref = None

state_lock = threading.Lock()
auto_mode = True
last_pan_cmd = None
last_tilt_cmd = None


def clamp(value, min_value, max_value):
	return max(min_value, min(max_value, value))


def format_axis_cmd(axis, velocity):
	sign = "+" if velocity >= 0 else "-"
	return f"{axis}{sign}{abs(int(velocity))}"


def kirim_perintah(cmd):
	if ser and ser.is_open:
		ser.write((cmd + "\n").encode())
		print(f"[KIRIM] {cmd}")


def set_axis_velocity(axis, velocity):
	global last_pan_cmd, last_tilt_cmd

	velocity = int(velocity)
	with state_lock:
		if axis == "P":
			if last_pan_cmd == velocity:
				return
			last_pan_cmd = velocity
		elif axis == "T":
			if last_tilt_cmd == velocity:
				return
			last_tilt_cmd = velocity
		else:
			return

	kirim_perintah(format_axis_cmd(axis, velocity))


def set_auto_mode(enabled):
	global auto_mode
	with state_lock:
		auto_mode = enabled
	mode_name = "AUTO" if enabled else "MANUAL"
	print(f"[MODE] {mode_name}")


def get_auto_mode():
	with state_lock:
		return auto_mode


def baca_serial():
	while not stop_event.is_set():
		if ser and ser.is_open and ser.in_waiting:
			try:
				data = ser.readline().decode(errors="ignore").strip()
				if data:
					print(f"  [STM32] {data}")
			except Exception:
				pass
		time.sleep(0.01)


def compute_auto_velocity(error, deadzone, gain, vmax):
	if abs(error) <= deadzone:
		return 0

	velocity = int(gain * error)
	return clamp(velocity, -vmax, vmax)


def toggle_mode():
	set_auto_mode(not get_auto_mode())
	set_axis_velocity("P", 0)
	set_axis_velocity("T", 0)


def saat_tekan(key):
	try:
		if key in keys_held:
			return
		keys_held.add(key)

		if key == keyboard.Key.right:
			set_auto_mode(False)
			set_axis_velocity("P", PAN_VEL_MANUAL)
		elif key == keyboard.Key.left:
			set_auto_mode(False)
			set_axis_velocity("P", -PAN_VEL_MANUAL)
		elif key == keyboard.Key.up:
			set_auto_mode(False)
			set_axis_velocity("T", TILT_VEL_MANUAL)
		elif key == keyboard.Key.down:
			set_auto_mode(False)
			set_axis_velocity("T", -TILT_VEL_MANUAL)
		elif hasattr(key, "char") and key.char:
			if key.char == "q":
				stop_event.set()
				return False
			if key.char == "s":
				kirim_perintah("S")
				set_axis_velocity("P", 0)
				set_axis_velocity("T", 0)
			elif key.char == "?":
				kirim_perintah("?")
			elif key.char == "1":
				kirim_perintah("IP")
			elif key.char == "2":
				kirim_perintah("IT")
			elif key.char == "m":
				toggle_mode()
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


def run_camera_loop():
	cap, cam_label = open_camera_auto()
	if cap is None:
		print("Kamera tidak ditemukan. Lanjut mode serial manual tanpa tampilan kamera.")
		return False

	print(f"Kamera aktif: {cam_label}")

	while not stop_event.is_set():
		ret, frm = cap.read()
		if not ret:
			break

		frm = cv2.flip(frm, 1)

		h, w = frm.shape[:2]
		cx_frm, cy_frm = w // 2, h // 2
		zx = cx_frm - w // 4
		zy = cy_frm - h // 4
		zw, zh = w // 2, h // 2

		hsv = cv2.cvtColor(frm, cv2.COLOR_BGR2HSV)
		msk = cv2.inRange(hsv, LOW_GREEN, HIGH_GREEN)
		msk = cv2.erode(msk, None, iterations=1)
		msk = cv2.dilate(msk, None, iterations=2)

		cnts, _ = cv2.findContours(msk, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		state_text = "NO TARGET"
		state_color = (0, 0, 255)
		err_x = 0
		err_y = 0
		target_found = False

		if cnts:
			c = max(cnts, key=cv2.contourArea)
			if cv2.contourArea(c) > MIN_CONTOUR_AREA:
				target_found = True
				x, y, bw, bh = cv2.boundingRect(c)
				cx = x + bw // 2
				cy = y + bh // 2
				err_x = cx - cx_frm
				err_y = cy_frm - cy

				cv2.rectangle(frm, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
				cv2.circle(frm, (cx, cy), 5, (0, 0, 255), -1)
				cv2.putText(
					frm,
					f"err:(PAN:{err_x:+d},TILT:{err_y:+d})",
					(10, 30),
					cv2.FONT_HERSHEY_SIMPLEX,
					0.6,
					(255, 255, 255),
					2,
				)

				if zx <= cx <= zx + zw and zy <= cy <= zy + zh:
					state_text = "STEADY"
					state_color = (0, 255, 0)
				else:
					state_text = "MOVING"
					state_color = (0, 165, 255)

		if get_auto_mode():
			if target_found:
				pan_velocity = compute_auto_velocity(err_x, DEADZONE_X, AUTO_GAIN, AUTO_MAX_PAN)
				tilt_velocity = compute_auto_velocity(err_y, DEADZONE_Y, AUTO_GAIN, AUTO_MAX_TILT)
				set_axis_velocity("P", pan_velocity)
				set_axis_velocity("T", tilt_velocity)
			else:
				set_axis_velocity("P", 0)
				set_axis_velocity("T", 0)

		mode_label = "AUTO" if get_auto_mode() else "MANUAL"

		cv2.rectangle(frm, (zx, zy), (zx + zw, zy + zh), (255, 255, 0), 1)
		cv2.putText(frm, state_text, (zx, zy - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)
		cv2.putText(
			frm,
			f"MODE: {mode_label} (m=toggle)",
			(10, h - 15),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.6,
			(255, 255, 255),
			2,
		)
		cv2.drawMarker(frm, (cx_frm, cy_frm), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)

		cv2.imshow("Green Tracking + Serial Control", frm)
		key = cv2.waitKey(1) & 0xFF
		if key == ord("q"):
			stop_event.set()
			break
		if key == ord("m"):
			toggle_mode()
		if key == ord("s"):
			kirim_perintah("S")
			set_axis_velocity("P", 0)
			set_axis_velocity("T", 0)

	cap.release()
	cv2.destroyAllWindows()
	return True


def open_camera_auto():
	device_paths = sorted(glob.glob("/dev/video*"))
	if not device_paths:
		return None, None

	candidate_indices = []

	for path in device_paths:
		match = re.search(r"/dev/video(\d+)$", path)
		if match:
			candidate_indices.append(int(match.group(1)))

	if not candidate_indices:
		return None, None

	for idx in candidate_indices:
		cap = cv2.VideoCapture(idx)
		if cap.isOpened():
			return cap, f"/dev/video{idx}"
		cap.release()

	return None, None


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

	print("=== Move Test (Camera + Serial) ===")
	print("Arrow keys : Manual gerak pan/tilt (otomatis pindah MANUAL)")
	print("m          : Toggle AUTO/MANUAL")
	print("s          : Stop semua")
	print("1          : Toggle invert pan")
	print("2          : Toggle invert tilt")
	print("?          : Query status")
	print("q          : Keluar")
	print("===================================")

	camera_available = run_camera_loop()

	if not camera_available:
		set_auto_mode(False)
		print("Menunggu input keyboard (tanpa kamera). Tekan q untuk keluar.")
		try:
			while not stop_event.is_set():
				time.sleep(0.1)
		except KeyboardInterrupt:
			stop_event.set()

	stop_event.set()
	set_axis_velocity("P", 0)
	set_axis_velocity("T", 0)
	kirim_perintah("S")

	if listener_ref is not None:
		listener_ref.stop()
		listener_ref.join(timeout=1)

	serial_thread.join(timeout=1)

	if ser and ser.is_open:
		ser.close()

	print("Selesai.")


if __name__ == "__main__":
	main()
