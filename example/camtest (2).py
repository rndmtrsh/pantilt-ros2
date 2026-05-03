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

    prev_bbox = None
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
            target_w = 480
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

            status = "NO FACE"
            color = (0, 0, 255)

            if len(faces) > 0:
                # use the largest face
                x, y, fw, fh = max(faces, key=lambda r: r[2] * r[3])
                detected = (float(x), float(y), float(fw), float(fh))
                if prev_bbox is None:
                    prev_bbox = detected
                else:
                    px, py, pw, ph = prev_bbox
                    nx = alpha * detected[0] + (1 - alpha) * px
                    ny = alpha * detected[1] + (1 - alpha) * py
                    nw = alpha * detected[2] + (1 - alpha) * pw
                    nh = alpha * detected[3] + (1 - alpha) * ph
                    prev_bbox = (nx, ny, nw, nh)

                bx, by, bw_s, bh_s = map(int, prev_bbox)
                cv2.rectangle(small, (bx, by), (bx + bw_s, by + bh_s), (0, 255, 0), 2)
                cv2.putText(small, "FACE", (bx, max(20, by - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                status = "DETECTED"
                color = (0, 255, 0)
            else:
                prev_bbox = None

            # FPS display
            now = time.time()
            fps = 1.0 / max(1e-6, now - last_time)
            last_time = now
            cv2.putText(small, f"FPS: {fps:.1f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(small, status, (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(gray_vis, "GRAY (EQUALIZED)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

            combined = np.hstack((gray_vis, small))
            cv2.imshow("Face Detection - Gray + Original", combined)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
