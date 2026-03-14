#!/usr/bin/python3

# ============================================================
# shared_state.py
# Variabel bersama antara target_detection_25.py
# dan control_25.py. Semua akses harus pakai lock.
#
# ALUR DATA:
#
#   mavlink_thread  →  shared_state  ←→  target_detection_25
#                           ↕
#                      control_25
# ============================================================

import threading

# Lock global — wajib dipakai setiap baca/tulis variabel di sini
lock = threading.Lock()

# ============================================================
# DATA DARI DRONE
# Diisi oleh: mavlink_data_update() di control_25.py
# Dibaca oleh: target_detection_25.py (untuk overlay)
#              control_25.py (untuk kalkulasi)
# ============================================================
alt                 = 0.0   # altitude relatif AGL (meter)
pitch               = 0.0   # sudut pitch (derajat)
groundspeed         = 0.0   # kecepatan terhadap tanah (m/s)
airspeed            = 0.0   # kecepatan terhadap udara (m/s)
wp_reached          = 0     # waypoint terakhir yang dicapai
rudder_out_pwm      = 0     # PWM output rudder (dari RC out)
payload_out_pwm     = 0     # PWM output payload servo (dari RC out)
throttle_percentage = 0.0   # persentase throttle 0-100
fcu_connected       = False # status koneksi FCU
fcu_mode            = ''    # mode FCU: 'AUTO', 'MANUAL', dll
current_latitude    = 0.0   # latitude GPS saat ini
current_longitude   = 0.0   # longitude GPS saat ini
current_heading     = 0.0   # heading pesawat (derajat, 0-360)
is_armed            = False # status armed/disarmed

# ============================================================
# OUTPUT DARI target_detection_25.py
# Diisi oleh: target_detection_25.py
# Dibaca oleh: control_25.py (untuk guidance & drop decision)
# ============================================================
target_x            = 0.5   # posisi lateral target (0.0=kiri, 1.0=kanan)
is_target_detected  = False # apakah target terdeteksi di frame

# ============================================================
# OUTPUT DARI control_25.py
# Diisi oleh: control_25.py
# Dibaca oleh: target_detection_25.py (untuk overlay di video)
# ============================================================
is_target_aligned   = False # apakah drone sudah lurus ke target
is_payload_dropped  = False # apakah payload sudah dijatuhkan
drop_travel_dist    = 999.0 # jarak tempuh payload saat jatuh bebas (meter)
coord_dist          = 999.0 # jarak GPS drone ke titik drop zone (meter)
centering_process   = False # apakah sedang dalam proses centering
running             = True  # flag untuk menghentikan semua thread