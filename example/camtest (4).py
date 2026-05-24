import cv2
from pathlib import Path
import time
import numpy as np
from ultralytics import YOLO


def main():
    script_dir = Path(__file__).resolve().parent
    model_path = script_dir / "best.pt"


    model = YOLO(str(model_path))
    target_class_name = "Human-body"
    target_class_id = None
    model_names = getattr(model, "names", None)
    if isinstance(model_names, dict):
        for class_id, name in model_names.items():
            if name == target_class_name:
                target_class_id = int(class_id)
                break
    elif isinstance(model_names, (list, tuple)):
        try:
            target_class_id = model_names.index(target_class_name)
        except ValueError:
            target_class_id = None
    if target_class_id is None:
        print(f"Class not found in model: {target_class_name}")
        raise SystemExit(1)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        raise SystemExit(1)

    # try to set a reasonable camera size (may be ignored by some cameras)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    prev_face_bbox = None
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

            yolo_input = small.copy()
            selected_vis = np.zeros_like(small)
            detected_vis = small.copy()

            results = model(small, imgsz=480, conf=0.25, verbose=False)
            boxes = results[0].boxes if results else None
            faces = []
            faces_conf = None
            if boxes is not None and len(boxes) > 0:
                boxes_xyxy = boxes.xyxy
                boxes_cls = boxes.cls
                boxes_conf = getattr(boxes, "conf", None)
                if hasattr(boxes_xyxy, "detach"):
                    boxes_xyxy = boxes_xyxy.detach().cpu().numpy()
                else:
                    boxes_xyxy = np.asarray(boxes_xyxy)
                if hasattr(boxes_cls, "detach"):
                    boxes_cls = boxes_cls.detach().cpu().numpy()
                else:
                    boxes_cls = np.asarray(boxes_cls)
                if boxes_conf is not None:
                    if hasattr(boxes_conf, "detach"):
                        boxes_conf = boxes_conf.detach().cpu().numpy()
                    else:
                        boxes_conf = np.asarray(boxes_conf)
                    boxes_conf = boxes_conf.reshape(-1)
                boxes_cls = boxes_cls.reshape(-1).astype(int)

                if boxes_xyxy.size > 0:
                    mask = boxes_cls == target_class_id
                    faces = boxes_xyxy[mask]
                    if boxes_conf is not None:
                        faces_conf = boxes_conf[mask]

            face_center = None
            raw_face_bbox = None
            selected_conf = None
            if len(faces) > 0:
                # use the largest detection
                areas = (faces[:, 2] - faces[:, 0]) * (faces[:, 3] - faces[:, 1])
                best_idx = int(np.argmax(areas))
                x1, y1, x2, y2 = faces[best_idx]
                raw_face_bbox = (float(x1), float(y1), float(x2 - x1), float(y2 - y1))
                if faces_conf is not None and len(faces_conf) > best_idx:
                    selected_conf = float(faces_conf[best_idx])

                rx, ry, rw, rh = map(int, raw_face_bbox)
                cv2.rectangle(detected_vis, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)
                conf_text = f"{selected_conf:.2f}" if selected_conf is not None else "HUMAN-BODY"
                cv2.putText(detected_vis, conf_text, (rx, max(20, ry - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                x1i = max(0, int(raw_face_bbox[0]))
                y1i = max(0, int(raw_face_bbox[1]))
                x2i = min(small.shape[1] - 1, int(raw_face_bbox[0] + raw_face_bbox[2]))
                y2i = min(small.shape[0] - 1, int(raw_face_bbox[1] + raw_face_bbox[3]))
                if x2i > x1i and y2i > y1i:
                    crop = yolo_input[y1i:y2i, x1i:x2i]
                    selected_vis[y1i:y2i, x1i:x2i] = crop
                    cv2.rectangle(selected_vis, (x1i, y1i), (x2i, y2i), (0, 255, 255), 2)

            if raw_face_bbox is not None:
                if prev_face_bbox is None:
                    prev_face_bbox = raw_face_bbox
                else:
                    px, py, pw, ph = prev_face_bbox
                    nx = alpha * raw_face_bbox[0] + (1 - alpha) * px
                    ny = alpha * raw_face_bbox[1] + (1 - alpha) * py
                    nw = alpha * raw_face_bbox[2] + (1 - alpha) * pw
                    nh = alpha * raw_face_bbox[3] + (1 - alpha) * ph
                    prev_face_bbox = (nx, ny, nw, nh)

                bx, by, bw_s, bh_s = map(int, prev_face_bbox)
                face_center = (bx + bw_s // 2, by + bh_s // 2)
                cv2.circle(detected_vis, face_center, 4, (0, 255, 0), -1)
            else:
                prev_face_bbox = None

            # frame center marker
            hs, ws = detected_vis.shape[:2]
            frame_center = (ws // 2, hs // 2)
            cv2.drawMarker(detected_vis, frame_center, (255, 0, 0), cv2.MARKER_CROSS, 18, 2)

            if face_center is not None:
                cv2.putText(
                    detected_vis,
                    f"TARGET: ({face_center[0]},{face_center[1]})",
                    (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 0, 255),
                    2,
                )
            else:
                cv2.putText(detected_vis, "TARGET WAIT", (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 2)

            # FPS display
            now = time.time()
            fps = 1.0 / max(1e-6, now - last_time)
            last_time = now
            cv2.putText(yolo_input, "RAW INPUT", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(selected_vis, "SELECTED PART", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(detected_vis, "DETECTED (CONF)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            if raw_face_bbox is None:
                cv2.putText(selected_vis, "NO SELECTION", (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 2)
            cv2.putText(detected_vis, f"FPS: {fps:.1f}", (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            combined = np.hstack((yolo_input, selected_vis, detected_vis))
            cv2.imshow("Human-body YOLO - Raw | Selected | Detected", combined)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
