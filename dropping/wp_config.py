#!/usr/bin/python3

# ============================================================
# wp_config.py
# Konstanta waypoint dan koordinat drop zone.
# Diimport oleh control_25.py DAN target_detection_25.py
# agar tidak terjadi circular import.
#
# CARA PAKAI:
#   Sesuaikan semua nilai di file ini dengan Mission Planner
#   sebelum terbang. Jangan ubah di file lain.
# ============================================================

# ============================================================
# WAYPOINT INDEX
# Sesuaikan dengan urutan WP di Mission Planner
# ============================================================

APPROACH_WP_INDEX_1 = 1   # WP saat pesawat masuk jalur drop zone 1
DROP_ZONE_WP_1      = 2   # WP akhir drop zone 1 (failsafe trigger)

APPROACH_WP_INDEX_2 = 3   # WP saat pesawat masuk jalur drop zone 2
DROP_ZONE_WP_2      = 4   # WP akhir drop zone 2 (failsafe trigger)

# ============================================================
# KOORDINAT DROP ZONE
# Ambil dari Mission Planner atau Google Maps
# ============================================================

# Titik awal approach (dipakai untuk kalkulasi bearing GPS)
LATITUDE_APPROACH_1,  LONGITUDE_APPROACH_1  = -7.7737333, 110.3785068
LATITUDE_APPROACH_2,  LONGITUDE_APPROACH_2  = -7.7735187, 110.3785685

# Titik target drop (pusat zona pendaratan payload)
LATITUDE_DROPZONE_1,  LONGITUDE_DROPZONE_1  = -7.7736224, 110.3785410
LATITUDE_DROPZONE_2,  LONGITUDE_DROPZONE_2  = -7.7734091, 110.3786007

# ============================================================
# ELEVASI DROP ZONE
# Ketinggian titik drop zone dari titik Home (meter)
# Ubah jika target berada di atas bukit atau gedung
# ============================================================

DROP_ZONE_ELEVATION = {
    DROP_ZONE_WP_1: 0.0,
    DROP_ZONE_WP_2: 0.0,
}

# ============================================================
# MAPPING
# ============================================================

# WP Approach → koordinat approach
APPROACH = {
    APPROACH_WP_INDEX_1: [LATITUDE_APPROACH_1, LONGITUDE_APPROACH_1],
    APPROACH_WP_INDEX_2: [LATITUDE_APPROACH_2, LONGITUDE_APPROACH_2],
}

# WP Drop → koordinat drop zone
DROPPING = {
    DROP_ZONE_WP_1: [LATITUDE_DROPZONE_1, LONGITUDE_DROPZONE_1],
    DROP_ZONE_WP_2: [LATITUDE_DROPZONE_2, LONGITUDE_DROPZONE_2],
}

# WP Approach → WP Drop tujuannya
APPROACH_TO_DROP_MAP = {
    APPROACH_WP_INDEX_1: DROP_ZONE_WP_1,
    APPROACH_WP_INDEX_2: DROP_ZONE_WP_2,
}