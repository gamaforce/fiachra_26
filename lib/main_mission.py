main_mission.py

```
#!/usr/bin/env python3

import logging
import time
import threading
import math
import numpy as np
from simple_pid import PID
from haversine import haversine, Unit
from pymavlink import mavutil
from ballistic_calc import calc_horizontal_travel_dist

# ================= KONFIGURASI GLOBAL =================
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')

PORT = '/dev/ttyACM0'
BAUDRATE = 115200
MODE = 'AUTO'

# ================= WAYPOINT & KOORDINAT =================
# GANTI DENGAN DATA REAL DARI MISSION PLANNER
APPROACH_WP_INDEX_1 = 1
DROP_ZONE_WP_1      = 2

APPROACH_WP_INDEX_2 = 3
DROP_ZONE_WP_2      = 4

# Koordinat (Latitude, Longitude)
LATITUDE_APPROACH_1, LONGITUDE_APPROACH_1 = -7.7737333, 110.3785068
LATITUDE_DROPZONE_1, LONGITUDE_DROPZONE_1 = -7.7736224, 110.3785410

LATITUDE_APPROACH_2, LONGITUDE_APPROACH_2 = -7.7735187, 110.3785685
LATITUDE_DROPZONE_2, LONGITUDE_DROPZONE_2 = -7.7734091, 110.3786007

# Mapping: WP Approach -> Koordinatnya
APPROACH = {
    APPROACH_WP_INDEX_1: [LATITUDE_APPROACH_1, LONGITUDE_APPROACH_1],
    APPROACH_WP_INDEX_2: [LATITUDE_APPROACH_2, LONGITUDE_APPROACH_2]
}

# Mapping: WP Drop -> Koordinatnya
DROPPING = {
    DROP_ZONE_WP_1: [LATITUDE_DROPZONE_1, LONGITUDE_DROPZONE_1],
    DROP_ZONE_WP_2: [LATITUDE_DROPZONE_2, LONGITUDE_DROPZONE_2]
}

# Mapping Pasangan: Jika lewat WP Approach X, maka target Drop adalah Y
APPROACH_TO_DROP_MAP = {
    APPROACH_WP_INDEX_1: DROP_ZONE_WP_1,
    APPROACH_WP_INDEX_2: DROP_ZONE_WP_2
}

# Elevation Target Drop (Ketinggian Drop Zone dari titik Home)
# Ini penting agar 'h' di rumus balistik akurat (h = Alt Pesawat - Elevasi Target)
DROP_ZONE_ELEVATION_LIST = {
    DROP_ZONE_WP_1: 0.0, # Ubah jika target ada di atas bukit/gedung
    DROP_ZONE_WP_2: 0.0
}

# ================= PARAMETER KONTROL =================
RUDDER_CHANNEL_INDEX = 4 # Sesuaikan (Aileron=1, Elevator=2, Throttle=3, Rudder=4)
DROP_MECHANISM_CHANNEL_INDEX = 7

DROP_MECHANISM_PWM = {
    DROP_ZONE_WP_1 : 1495,
    DROP_ZONE_WP_2 : 970
}

ACCEPTABLE_DEVIATION = 0.05 # Derajat toleransi lurus
yaw_pid = PID(Kp=12, Ki=0, Kd=0.11, setpoint=0, output_limits=(-400, 400))

master = None 

# ================= SHARED MEMORY =================
class SharedState:
    def __init__(self):
        self.lock = threading.Lock()

        # Flags
        self.running = True
        self.connection_status = False
        self.centering_process = False
        self.is_payload_dropped = False
        self.is_aligned = False

        # Vision Data
        self.is_target_detected = False
        self.target_x = 0.5 

        # FC Data
        self.wp_reached = 0
        self.groundspeed = 0.0
        self.airspeed = 0.0
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        self.current_mode = 'UNKNOWN'
        self.is_armed = False
        self.current_heading = 0.0

# ================= HELPER FUNCTIONS =================
def coordinate_angle(lat1, lon1, lat2, lon2):
    """Menghitung Bearing dalam Derajat (0-360)"""
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dLon = lon2 - lon1
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360

def set_override_pwm(state, rudder_pwm, dropping_wp_index):
    global master
    
    # 1. Rudder Control (RC Override)
    # Gunakan RC Override agar tidak konflik dengan flight controller di mode AUTO
    if rudder_pwm != 0:
        rc_vals = [65535] * 8 # 65535 = Ignore
        if 1 <= RUDDER_CHANNEL_INDEX <= 8:
            rc_vals[RUDDER_CHANNEL_INDEX - 1] = int(rudder_pwm)
            master.mav.rc_channels_override_send(
                master.target_system, master.target_component, *rc_vals
            )

    # 2. Payload Release (DO_SET_SERVO)
    # Hanya kirim jika flag drop sudah aktif
    if state.is_payload_dropped:
        target_pwm = DROP_MECHANISM_PWM.get(dropping_wp_index, 1500)
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            DROP_MECHANISM_CHANNEL_INDEX,
            target_pwm,
            0, 0, 0, 0, 0
        )
        state.centering_process = False
        logging.info(f"💣 SERVO COMMAND SENT: PWM {target_pwm}")

def trigger_drop(state, dropping_area_index):
    state.is_payload_dropped = True
    set_override_pwm(state, 0, dropping_area_index)
    logging.info(f"===== PAYLOAD RELEASED (TRIGGER) =====")

def failsafe_drop(state, dropping_area_index):
    state.is_payload_dropped = True
    set_override_pwm(state, 0, dropping_area_index)
    logging.info(f"===== PAYLOAD RELEASED (FAILSAFE) =====")

# ================= THREAD: MAVLINK UPDATER =================
def mavlink_data_update(state):
    global master
    logging.info('[THREAD] Data update thread started')

    try:
        master = mavutil.mavlink_connection(PORT, baud=BAUDRATE)
        master.wait_heartbeat()
        logging.info('[THREAD] Heartbeat received')
        with state.lock:
            state.connection_status = True
    except Exception as e:
        logging.error(f'[THREAD] Connection error: {e}')
        return

    while state.running:
        msg = master.recv_match(type=['HEARTBEAT', 'MISSION_ITEM_REACHED', 'VFR_HUD', 'GLOBAL_POSITION_INT'], blocking=False)

        if not msg:
            time.sleep(0.01)
            continue
            
        msg_type = msg.get_type()

        with state.lock:
            if msg_type == 'HEARTBEAT':
                state.current_mode = master.flightmode
                state.is_armed = master.motors_armed()
            
            elif msg_type == 'MISSION_ITEM_REACHED':
                state.wp_reached = msg.seq # WP yang baru saja dilewati
                logging.info(f"📍 PASSED WP: {state.wp_reached}")

            elif msg_type == 'GLOBAL_POSITION_INT':
                state.current_lat = msg.lat / 1e7
                state.current_lon = msg.lon / 1e7
                state.current_alt = msg.relative_alt / 1000.0 # Relative Altitude (AGL)
                if msg.hdg != 65535:
                    state.current_heading = msg.hdg / 100.0
        
            elif msg_type == 'VFR_HUD':
                state.groundspeed = msg.groundspeed
                state.airspeed = msg.airspeed
        
        time.sleep(0.02) 

# ================= MAIN PROGRAM =================
def main():
    logging.info('===== MISSION START =====')
    state = SharedState()
    
    mavlink_thread = threading.Thread(target=mavlink_data_update, args=(state,))
    mavlink_thread.start()

    while not state.connection_status:
        time.sleep(1)
        logging.info("Waiting for FC connection...")

    logging.info('SYSTEM READY. Waiting for Approach WP...')
    
    current_drop_target = -1

    try:
        while True:
            time.sleep(0.1) # 10Hz Control Loop

            if state.connection_status:
                
                # Copy data state agar thread-safe
                with state.lock:
                    wp_passed = state.wp_reached
                    lat = state.current_lat
                    lon = state.current_lon
                    alt = state.current_alt
                    gs = state.groundspeed
                    ias = state.airspeed
                    mode = state.current_mode
                    detected = state.is_target_detected
                    tx = state.target_x
                    dropped = state.is_payload_dropped
                    hdg = state.current_heading

                # --- 1. LOGIKA RESET FLAG (PRE-APPROACH) ---
                # Jika WP yang baru lewat adalah "Pre-Approach" (Approach - 1)
                # Maka kita reset flag agar siap dropping di cycle ini
                if (wp_passed + 1) in APPROACH:
                    if dropped:
                        logging.info("Resetting drop flag for next run.")
                        with state.lock: state.is_payload_dropped = False

                # --- 2. LOGIKA APPROACH & DROP (PASSED APPROACH WP) ---
                # Jika kita BARU SAJA melewati WP Approach, berarti kita sedang OTW Drop Zone
                if wp_passed in APPROACH and mode == 'AUTO':
                    with state.lock: state.centering_process = True
                    
                    # Identifikasi Target Drop Zone
                    current_drop_target = APPROACH_TO_DROP_MAP[wp_passed]
                    
                    # Parameter Balistik
                    elevation_ref = DROP_ZONE_ELEVATION_LIST.get(current_drop_target, 0)
                    h_effective = alt - elevation_ref
                    
                    # Hitung Jarak Tempuh Balistik (Horizontal)
                    # Memanggil fungsi dari free_fall3.py
                    # Parameter: groundspeed, height, airspeed, drop_index (opsional)
                    drop_travel_dist = calc_horizontal_travel_dist(gs, h_effective, ias, current_drop_target)
                    
                    # Hitung Jarak Real-time ke Target
                    current_loc = (lat, lon)
                    target_loc = tuple(DROPPING[current_drop_target])
                    coordinate_distance = haversine(current_loc, target_loc, unit=Unit.METERS)

                    # --- GUIDANCE CONTROL (Rudder) ---
                    rudder_pwm = 1500
                    
                    if detected:
                        # Vision Based Guidance
                        deviation = tx - 0.5
                        rudder_pwm = 1500 + int(yaw_pid(deviation))
                        logging.info(f'VISION | Dev: {deviation:.2f} | PWM: {rudder_pwm}')
                        
                        if abs(deviation) < abs(ACCEPTABLE_DEVIATION):
                            with state.lock: state.is_aligned = True
                        else: 
                            with state.lock: state.is_aligned = False
                        
                        set_override_pwm(state, rudder_pwm, current_drop_target)
                        
                    else:
                        # GPS Based Guidance
                        approach_coord = APPROACH[wp_passed]
                        drop_coord = DROPPING[current_drop_target]
                        
                        # Hitung Sudut Jalur (Track) & Sudut Pesawat
                        track_angle = coordinate_angle(approach_coord[0], approach_coord[1], drop_coord[0], drop_coord[1])
                        plane_angle = coordinate_angle(lat, lon, drop_coord[0], drop_coord[1])
                            
                        # Hitung Error Sudut
                        deviation_angle = (track_angle - plane_angle + 180) % 360 - 180
                        
                        # Normalisasi Error (-1.0 s/d 1.0) untuk PID
                        error_norm = np.clip(deviation_angle / 45.0, -1.0, 1.0)
                        rudder_pwm = 1500 + int(yaw_pid(error_norm))

                        if (abs(deviation_angle) < abs(ACCEPTABLE_DEVIATION)) and state.centering_process:
                            with state.lock: state.is_aligned = True
                        else: 
                            with state.lock: state.is_aligned = False
                            
                        set_override_pwm(state, rudder_pwm, current_drop_target)

                    # --- EXECUTION: DROPPING ---
                    # Jika jarak pesawat ke target <= jarak hitungan balistik
                    if coordinate_distance <= drop_travel_dist and not dropped:
                        logging.info(f"===== DROP CONDITION MET! Dist: {coordinate_distance:.2f} <= {drop_travel_dist:.2f} =====")
                        trigger_drop(state, current_drop_target)
                        
                # --- 3. FAILSAFE DROP ---
                # Jika kita melewati WP Drop Zone tapi payload belum jatuh
                elif wp_passed in DROPPING and not dropped:
                     # Pastikan WP ini adalah target dari approach terakhir
                     if wp_passed == current_drop_target:
                        logging.warning(f"===== FAILSAFE: Missed calculation at WP {wp_passed} =====")
                        failsafe_drop(state, wp_passed)
                        current_drop_target = -1 # Reset cycle

            else:
                # Jika koneksi putus
                logging.info('Waiting for FC connection...')

    except KeyboardInterrupt:
        logging.info('User stopped mission loop.')
        state.running = False
        mavlink_thread.join()
        logging.info('MAVLink thread stopped.')

if __name__ == '__main__':
    main()
```
