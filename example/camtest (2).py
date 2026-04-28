import cv2
from pathlib import Path

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    exit()

script_dir = Path(__file__).resolve().parent
cascade_candidates = [
    script_dir.parent / "haarcascade_upperbody.xml",
    script_dir / "haarcascade_upperbody.xml",
    Path("haarcascade_upperbody.xml"),
]

upper_body_cascade = None
for cascade_path in cascade_candidates:
    cascade = cv2.CascadeClassifier(str(cascade_path))
    if not cascade.empty():
        upper_body_cascade = cascade
        break

if upper_body_cascade is None:
    cap.release()
    raise SystemExit("Could not load haarcascade_upperbody.xml")

while True:
    ret, frm = cap.read()
    if not ret:
        break

    frm = cv2.flip(frm, 1)
    gray = cv2.cvtColor(frm, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)

    bodies = upper_body_cascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(60, 60),
    )

    status = "NO UPPER BODY"
    color = (0, 0, 255)

    if len(bodies) > 0:
        x, y, bw, bh = max(bodies, key=lambda rect: rect[2] * rect[3])
        cv2.rectangle(frm, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        cv2.putText(frm, "UPPER BODY", (x, max(25, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        status = f"DETECTED: {len(bodies)}"
        color = (0, 255, 0)

    cv2.putText(frm, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    cv2.imshow("Upper Body Detection", frm)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
