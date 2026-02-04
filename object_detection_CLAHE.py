import cv2
import numpy as np

cap = cv2.VideoCapture(1)  # 0 = kamera default

if not cap.isOpened():
    print("Kamera tidak terbuka")
    exit()

clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))

def side_length(p1, p2):
    return np.linalg.norm(p1 - p2)

def angle_between(v1, v2):
    cosang = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    cosang = np.clip(cosang, -1.0, 1.0) 
    return np.degrees(np.arccos(cosang))

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow('Original Image', frame)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    gray_eq = clahe.apply(gray)

    blur = cv2.GaussianBlur(gray_eq, (5, 5), 0)

    median_value = np.median(blur)

    lower = int(max(0, 0.7 * median_value))
    upper = int(min(255, 1.4 * median_value))

    edges = cv2.Canny(blur, lower, upper)
    cv2.imshow('Canny Edges', edges)

    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(
        edges,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    for cnt in contours:
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

        if len(approx) != 4:
            continue

        area = cv2.contourArea(approx)
        frame_ratio = frame.shape[0] * frame.shape[1]
        area_ratio = area / frame_ratio

        if area_ratio < 0.00005:  # filter area yang terlalu kecil
            continue

        pts = approx.reshape(4, 2)

        # hitung panjang sisi

        sides = [
            side_length(pts[0], pts[1]),
            side_length(pts[1], pts[2]),
            side_length(pts[2], pts[3]),
            side_length(pts[3], pts[0])
        ]

        longest = max(sides)
        shortest = min(sides)

        aspect_ratio = longest / shortest

        if aspect_ratio > 6:
            continue

        pts = approx.reshape(4, 2)

        right_angles = True

        for i in range(4):
            p_prev = pts[i - 1]
            p_curr = pts[i]
            p_next = pts[(i + 1) % 4]

            v1 = p_prev - p_curr
            v2 = p_next - p_curr

            angle = angle_between(v1, v2)

            if angle < 80 or angle > 100:
                right_angles = False
                break

        if not right_angles:
            continue

        cv2.drawContours(frame, [approx], -1, (0, 255, 0), 3)

        x, y = approx[0][0]
        cv2.putText(
            frame,
            "SEGI EMPAT",
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

    cv2.imshow("Edges Final", edges)
    cv2.imshow("Deteksi Segi Empat", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
