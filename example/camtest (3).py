import cv2
from pathlib import Path
import time
import numpy as np


def main():
    script_dir = Path(__file__).resolve().parent
    cascade_path = script_dir / "haarcascade_frontalface_alt2.xml"
    if not cascade_path.exists():
        print(f"Cascade not found: {cascade_path}")
        raise SystemExit(1)

    face_cascade = cv2.CascadeClassifier(str(cascade_path))
    if face_cascade.empty():
        print("Failed to load cascade")
        raise SystemExit(1)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        raise SystemExit(1)

    # try to set a reasonable camera size (may be ignored by some cameras)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    prev_face_bbox = None
    prev_color_center = None
    alpha = 0.5  # smoothing factor (0=no smoothing, 1=instant)
    last_time = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            frame = cv2.flip(frame, 1)

            # resize for faster detection while keeping aspect ratio
            target_w = 420
            h, w = frame.shape[:2]
            scale = target_w / float(w)
            small = cv2.resize(frame, (target_w, int(h * scale)))

            gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)
            gray_vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            faces = face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30),
            )

            # green color detection in HSV
            hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
            low_green = np.array([35, 80, 80], dtype=np.uint8)
            high_green = np.array([85, 255, 255], dtype=np.uint8)
            color_mask = cv2.inRange(hsv, low_green, high_green)
            color_mask = cv2.erode(color_mask, None, iterations=1)
            color_mask = cv2.dilate(color_mask, None, iterations=2)
            contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            color_vis = cv2.cvtColor(color_mask, cv2.COLOR_GRAY2BGR)
            cv2.putText(color_vis, "COLOR MASK (GREEN)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

            face_status = "NO FACE"
            face_color = (0, 0, 255)
            color_status = "NO COLOR"
            color_status_color = (0, 0, 255)

            face_center = None
            color_center = None

            if len(faces) > 0:
                # use the largest face
                x, y, fw, fh = max(faces, key=lambda r: r[2] * r[3])
                detected = (float(x), float(y), float(fw), float(fh))
                if prev_face_bbox is None:
                    prev_face_bbox = detected
                else:
                    px, py, pw, ph = prev_face_bbox
                    nx = alpha * detected[0] + (1 - alpha) * px
                    ny = alpha * detected[1] + (1 - alpha) * py
                    nw = alpha * detected[2] + (1 - alpha) * pw
                    nh = alpha * detected[3] + (1 - alpha) * ph
                    prev_face_bbox = (nx, ny, nw, nh)

                bx, by, bw_s, bh_s = map(int, prev_face_bbox)
                cv2.rectangle(small, (bx, by), (bx + bw_s, by + bh_s), (0, 255, 0), 2)
                cv2.putText(small, "FACE", (bx, max(20, by - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                face_center = (bx + bw_s // 2, by + bh_s // 2)
                cv2.circle(small, face_center, 4, (0, 255, 0), -1)
                face_status = "FACE DETECTED"
                face_color = (0, 255, 0)
            else:
                prev_face_bbox = None

            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 400:
                    x, y, bw, bh = cv2.boundingRect(c)
                    detected_color_center = (float(x + bw // 2), float(y + bh // 2))

                    if prev_color_center is None:
                        prev_color_center = detected_color_center
                    else:
                        pcx, pcy = prev_color_center
                        ncx = alpha * detected_color_center[0] + (1 - alpha) * pcx
                        ncy = alpha * detected_color_center[1] + (1 - alpha) * pcy
                        prev_color_center = (ncx, ncy)

                    ccx, ccy = map(int, prev_color_center)
                    color_center = (ccx, ccy)

                    cv2.rectangle(small, (x, y), (x + bw, y + bh), (0, 255, 255), 2)
                    cv2.circle(small, color_center, 4, (0, 255, 255), -1)
                    cv2.putText(small, "COLOR", (x, max(20, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                    cv2.rectangle(color_vis, (x, y), (x + bw, y + bh), (0, 255, 255), 2)
                    cv2.circle(color_vis, color_center, 4, (0, 255, 255), -1)
                    color_status = "COLOR DETECTED"
                    color_status_color = (0, 255, 0)
                else:
                    prev_color_center = None
            else:
                prev_color_center = None

            # frame center marker
            hs, ws = small.shape[:2]
            frame_center = (ws // 2, hs // 2)
            cv2.drawMarker(small, frame_center, (255, 0, 0), cv2.MARKER_CROSS, 18, 2)

            if face_center is not None and color_center is not None:
                mid_x = (face_center[0] + color_center[0]) // 2
                mid_y = (face_center[1] + color_center[1]) // 2
                fused_center = (mid_x, mid_y)
                cv2.line(small, face_center, color_center, (255, 255, 0), 2)
                cv2.circle(small, fused_center, 6, (255, 0, 255), -1)
                cv2.putText(
                    small,
                    f"TARGET MID: ({mid_x},{mid_y})",
                    (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 0, 255),
                    2,
                )
            else:
                cv2.putText(small, "TARGET WAIT", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 2)

            # FPS display
            now = time.time()
            fps = 1.0 / max(1e-6, now - last_time)
            last_time = now
            cv2.putText(small, f"FPS: {fps:.1f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(small, face_status, (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, face_color, 2)
            cv2.putText(small, color_status, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_status_color, 2)
            cv2.putText(gray_vis, "GRAY (EQUALIZED)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

            combined = np.hstack((gray_vis, color_vis, small))
            cv2.imshow("Face + Color Detection - Gray | Mask | Original", combined)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
