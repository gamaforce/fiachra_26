# import cv2
# import numpy as np

# LOOP_RATE = 30
# ENCODED_FPS = 30

# MIN_THROTTLE = 900
# MAX_THROTTLE = 2200

# DROP_ZONE_START_WP_1 = 5
# DROP_ZONE_END_WP_1 = 6
# DROP_ZONE_START_WP_2 = 41 #23
# DROP_ZONE_END_WP_2 = 42 #24


# APPROACH_DROPPING_WP_INDEX_1 = DROP_ZONE_START_WP_1
# APPROACH_DROPPING_WP_INDEX_2 = DROP_ZONE_START_WP_2
# APPROACH_DROPPING_WP_INDEX = [APPROACH_DROPPING_WP_INDEX_1, APPROACH_DROPPING_WP_INDEX_2]

# # sudut penempatan kamera
# CAM_ANGLE = 135

# RECORD_POST_EFFECT = True


# # elevasi drop zone terhadap gcs
# DROP_ZONE_ELEVATION_1 = 0 #-93.13
# DROP_ZONE_ELEVATION_2 = 0 #-89.698

# DROP_ZONE_ELEVATION_LIST = [DROP_ZONE_ELEVATION_1, DROP_ZONE_ELEVATION_2]

# cap = cv2.VideoCapture(0)  # 0 = kamera default

# if not cap.isOpened():
#     print("Kamera tidak terbuka")
#     exit()

# clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
# kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))

# def side_length(p1, p2):
#     return np.linalg.norm(p1 - p2)

# def angle_between(v1, v2):
#     cosang = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
#     cosang = np.clip(cosang, -1.0, 1.0) 
#     return np.degrees(np.arccos(cosang))

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         break

#     cv2.imshow('Original Image', frame)

#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     gray_eq = clahe.apply(gray)

#     blur = cv2.GaussianBlur(gray_eq, (5, 5), 0)

#     median_value = np.median(blur)

#     lower = int(max(0, 0.7 * median_value))
#     upper = int(min(255, 1.4 * median_value))

#     edges = cv2.Canny(blur, lower, upper)
#     cv2.imshow('Canny Edges', edges)

#     edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)

#     contours, _ = cv2.findContours(
#         edges,
#         cv2.RETR_EXTERNAL,
#         cv2.CHAIN_APPROX_SIMPLE
#     )

#     for cnt in contours:
#         peri = cv2.arcLength(cnt, True)
#         approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

#         if len(approx) != 4:
#             continue

#         area = cv2.contourArea(approx)
#         frame_ratio = frame.shape[0] * frame.shape[1]
#         area_ratio = area / frame_ratio

#         if area_ratio < 0.00005:  # filter area yang terlalu kecil
#             continue

#         pts = approx.reshape(4, 2)

#         # hitung panjang sisi

#         sides = [
#             side_length(pts[0], pts[1]),
#             side_length(pts[1], pts[2]),
#             side_length(pts[2], pts[3]),
#             side_length(pts[3], pts[0])
#         ]

#         longest = max(sides)
#         shortest = min(sides)

#         aspect_ratio = longest / shortest

#         if aspect_ratio > 6:
#             continue

#         pts = approx.reshape(4, 2)

#         right_angles = True

#         for i in range(4):
#             p_prev = pts[i - 1]
#             p_curr = pts[i]
#             p_next = pts[(i + 1) % 4]

#             v1 = p_prev - p_curr
#             v2 = p_next - p_curr

#             angle = angle_between(v1, v2)

#             if angle < 80 or angle > 100:
#                 right_angles = False
#                 break

#         if not right_angles:
#             continue

#         cv2.drawContours(frame, [approx], -1, (0, 255, 0), 3)

#         x, y = approx[0][0]
#         cv2.putText(
#             frame,
#             "SEGI EMPAT",
#             (x, y - 10),
#             cv2.FONT_HERSHEY_SIMPLEX,
#             0.6,
#             (0, 255, 0),
#             2
#         )

#     cv2.imshow("Edges Final", edges)
#     cv2.imshow("Deteksi Segi Empat", frame)

#     if cv2.waitKey(1) & 0xFF == 27:  # ESC
#         break

# cap.release()
# cv2.destroyAllWindows()


#!/usr/bin/python3

# ============================================================
# target_detection_25.py
# Deteksi target segi empat menggunakan kamera
# Versi pymavlink (tanpa ROS)
# ============================================================

import cv2
import numpy as np
import time
from datetime import datetime
import shared_state
from wp_config import (
    APPROACH_WP_INDEX_1, APPROACH_WP_INDEX_2,
    DROP_ZONE_WP_1, DROP_ZONE_WP_2
)

# ============================================================
# KONSTANTA KONFIGURASI
# ============================================================

LOOP_RATE    = 30
ENCODED_FPS  = 30

# Rekam video setelah overlay (True) atau sebelum (False)
RECORD_POST_EFFECT = True

# Deteksi aktif dari WP approach sampai WP drop selesai
DETECTION_WP_START_1 = APPROACH_WP_INDEX_1
DETECTION_WP_END_1   = DROP_ZONE_WP_1

DETECTION_WP_START_2 = APPROACH_WP_INDEX_2
DETECTION_WP_END_2   = DROP_ZONE_WP_2

# ============================================================
# FUNGSI HELPER DETEKSI
# ============================================================

def side_length(p1, p2):
    """Hitung panjang sisi antara dua titik"""
    return np.linalg.norm(p1 - p2)

def angle_between(v1, v2):
    """Hitung sudut antara dua vektor (derajat)"""
    cosang = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    cosang = np.clip(cosang, -1.0, 1.0)
    return np.degrees(np.arccos(cosang))

def detect_rectangle(frame):
    """
    Deteksi segi empat dalam frame.
    Return: (cx, cy, approx) jika terdeteksi, atau None jika tidak
    """
    gray     = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_eq  = clahe.apply(gray)
    blur     = cv2.GaussianBlur(gray_eq, (5, 5), 0)

    # Threshold canny otomatis berdasarkan median
    median_value = np.median(blur)
    lower = int(max(0,   0.7 * median_value))
    upper = int(min(255, 1.4 * median_value))
    edges = cv2.Canny(blur, lower, upper)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, morph_kernel, iterations=2)

    contours, _ = cv2.findContours(
        edges,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    for cnt in contours:
        peri  = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

        # Filter: harus 4 sisi
        if len(approx) != 4:
            continue

        # Filter: area minimum
        area        = cv2.contourArea(approx)
        frame_ratio = frame.shape[0] * frame.shape[1]
        area_ratio  = area / frame_ratio
        if area_ratio < 0.00005:
            continue

        # Filter: aspect ratio (tidak terlalu memanjang)
        pts = approx.reshape(4, 2)
        sides = [
            side_length(pts[0], pts[1]),
            side_length(pts[1], pts[2]),
            side_length(pts[2], pts[3]),
            side_length(pts[3], pts[0])
        ]
        longest  = max(sides)
        shortest = min(sides)
        if shortest == 0 or (longest / shortest) > 6:
            continue

        # Filter: sudut harus mendekati 90 derajat
        right_angles = True
        for i in range(4):
            p_prev = pts[i - 1]
            p_curr = pts[i]
            p_next = pts[(i + 1) % 4]
            v1    = p_prev - p_curr
            v2    = p_next - p_curr
            angle = angle_between(v1, v2)
            if angle < 80 or angle > 100:
                right_angles = False
                break

        if not right_angles:
            continue

        # Hitung titik tengah kontur
        M = cv2.moments(approx)
        if M['m00'] == 0:
            continue
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        return cx, cy, approx, edges

    return None, None, None, edges

# ============================================================
# INISIALISASI
# ============================================================

print('[target_detection] Initializing...')

# Inisialisasi kamera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print('[target_detection] ERROR: Kamera tidak terbuka')
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

frame_width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f'[target_detection] Kamera siap: {frame_width}x{frame_height}')

# Inisialisasi CLAHE dan kernel morfologi
clahe        = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

# Inisialisasi video recorder
fourcc = cv2.VideoWriter_fourcc(*'PIM1')
out    = cv2.VideoWriter(
    datetime.now().strftime('%Y-%m-%d_%H_%M_%S') + '.avi',
    fourcc,
    ENCODED_FPS,
    (frame_width, frame_height)
)
print('[target_detection] Video recorder siap')
print('[target_detection] Initialization complete')

# ============================================================
# MAIN LOOP
# ============================================================

while True:
    loop_start = time.time()

    # Baca frame dari kamera
    ret, frame = cap.read()
    if not ret:
        print('[target_detection] ERROR: Kamera tertutup, mencoba membuka ulang...')
        cap = cv2.VideoCapture(0)
        continue

    # Flip frame (kamera terbalik)
    frame = cv2.flip(frame, 1)

    # Rekam frame SEBELUM overlay jika RECORD_POST_EFFECT = False
    if not RECORD_POST_EFFECT:
        out.write(frame)

    # Baca variabel drone dari shared_state
    with shared_state.lock:
        wp_reached          = shared_state.wp_reached
        alt                 = shared_state.alt
        pitch               = shared_state.pitch
        groundspeed         = shared_state.groundspeed
        airspeed            = shared_state.airspeed
        rudder_out_pwm      = shared_state.rudder_out_pwm
        payload_out_pwm     = shared_state.payload_out_pwm
        throttle_percentage = shared_state.throttle_percentage
        fcu_connected       = shared_state.fcu_connected
        fcu_mode            = shared_state.fcu_mode
        is_target_aligned   = shared_state.is_target_aligned
        drop_travel_dist    = shared_state.drop_travel_dist
        coord_dist          = shared_state.coord_dist
        is_payload_dropped  = shared_state.is_payload_dropped

    # --------------------------------------------------------
    # CEK APAKAH DETEKSI PERLU DIAKTIFKAN
    # --------------------------------------------------------
    detection_active = (
        (DETECTION_WP_START_1 <= wp_reached <= DETECTION_WP_END_1 and fcu_mode == "AUTO") or
        (DETECTION_WP_START_2 <= wp_reached <= DETECTION_WP_END_2 and fcu_mode == "AUTO")
    )

    if detection_active:
        # --------------------------------------------------------
        # JALANKAN DETEKSI SEGI EMPAT
        # --------------------------------------------------------
        cx, cy, approx, edges = detect_rectangle(frame)

        if cx is not None:
            # Target terdeteksi
            print(f'[target_detection] Target terdeteksi di ({cx},{cy})')

            # Tulis hasil ke shared_state
            with shared_state.lock:
                shared_state.target_x           = cx / frame_width
                shared_state.is_target_detected = True

            # Gambar kontur & titik tengah
            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 3)
            cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)

            # Label deteksi
            cv2.putText(frame, 'DETECTED: TRUE', (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f'TARGET X: {cx/frame_width:.2f}', (cx+10, cy+5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Status alignment
            if is_target_aligned:
                cv2.putText(frame, 'TARGET ALIGNED: TRUE', (10, 300),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            else:
                cv2.putText(frame, 'TARGET ALIGNED: FALSE', (10, 300),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Drop travel distance
            cv2.putText(frame, f'DROP TRAVEL DIST: {drop_travel_dist:.2f}', (10, 320),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f'COORD DIST: {coord_dist:.2f} m', (10, 340),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            cv2.putText(frame, f'PAYLOAD DROPPED: {is_payload_dropped}', (10, 360),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0) if is_payload_dropped else (0, 0, 255), 2)

        else:
            # Target tidak terdeteksi
            with shared_state.lock:
                shared_state.is_target_detected = False

            cv2.putText(frame, 'DETECTED: FALSE', (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Tampilkan hasil edge detection
        cv2.imshow('Canny Edges', edges)

    else:
        # Deteksi tidak aktif di luar drop zone
        with shared_state.lock:
            shared_state.is_target_detected = False

        cv2.putText(frame, 'DETECTED: FALSE', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # --------------------------------------------------------
    # OVERLAY TELEMETRI (selalu tampil di setiap frame)
    # --------------------------------------------------------

    # Status FCU
    if fcu_connected:
        cv2.putText(frame, f'MODE: {fcu_mode}', (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    else:
        cv2.putText(frame, 'MODE: DISCONNECTED', (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Status payload
    if payload_out_pwm == 1495:
        cv2.putText(frame, 'PAYLOAD 1 DROPPED: TRUE',  (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(frame, 'PAYLOAD 2 DROPPED: FALSE', (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    elif payload_out_pwm == 875:
        cv2.putText(frame, 'PAYLOAD 1 DROPPED: TRUE', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(frame, 'PAYLOAD 2 DROPPED: TRUE', (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Data telemetri
    cv2.putText(frame, f'REACHED WP: {wp_reached}',            (10, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame, f'ALTITUDE: {alt:.2f} m',               (10, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame, f'GROUNDSPEED: {groundspeed:.2f} m/s',  (10, 140),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame, f'AIRSPEED: {airspeed:.2f} m/s',        (10, 160),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame, f'RUDDER PWM: {rudder_out_pwm:.0f}',    (10, 180),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame, f'PITCH: {pitch:.2f} deg',              (10, 200),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame, f'THROTTLE: {throttle_percentage:.1f}%',(10, 220),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # --------------------------------------------------------
    # REKAM & TAMPILKAN FRAME
    # --------------------------------------------------------

    # Rekam frame SETELAH overlay jika RECORD_POST_EFFECT = True
    if RECORD_POST_EFFECT:
        out.write(frame)

    cv2.imshow('Target Detection', frame)

    # Tekan ESC untuk keluar
    if cv2.waitKey(1) & 0xFF == 27:
        break

    # Jaga loop rate
    elapsed = time.time() - loop_start
    sleep_time = (1.0 / LOOP_RATE) - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)

# ============================================================
# CLEANUP
# ============================================================

print('[target_detection] Shutting down...')
cap.release()
out.release()
cv2.destroyAllWindows()
print('[target_detection] Shutdown complete')