import cv2
import numpy as np

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    exit()

while True:
    ret, frm = cap.read()
    if not ret:
        break
    
    frm = cv2.flip(frm, 1) 
    
    h, w = frm.shape[:2]
    cx_frm, cy_frm = w // 2, h // 2
    
    hsv = cv2.cvtColor(frm, cv2.COLOR_BGR2HSV)
    
    low = np.array([35, 80, 80])
    high = np.array([85, 255, 255])
    msk = cv2.inRange(hsv, low, high)
    
    msk = cv2.erode(msk, None, iterations=1)
    msk = cv2.dilate(msk, None, iterations=2)
    
    cnts, _ = cv2.findContours(msk, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    zx = cx_frm - w // 4
    zy = cy_frm - h // 4
    zw, zh = w // 2, h // 2
    
    state = "NO TARGET"
    state_color = (0, 0, 255)
    
    if cnts:
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) > 500:
            x, y, bw, bh = cv2.boundingRect(c)
            cx = x + bw // 2
            cy = y + bh // 2
            err_x = cx - cx_frm
            err_y = cy_frm - cy
            
            cv2.rectangle(frm, (x, y), (x+bw, y+bh), (0, 255, 0), 2)
            cv2.circle(frm, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(frm, f"err:(PAN:{err_x:+d},TILT:{err_y:+d})", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            if zx <= cx <= zx + zw and zy <= cy <= zy + zh:
                state = "STEADY"
                state_color = (0, 255, 0)
            else:
                state = "MOVING"
                state_color = (0, 165, 255)
    
    cv2.rectangle(frm, (zx, zy), (zx+zw, zy+zh), (255, 255, 0), 1)
    cv2.putText(frm, state, (zx, zy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)
    
    cv2.drawMarker(frm, (cx_frm, cy_frm), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
    
    cv2.imshow("Green Tracking", frm)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
