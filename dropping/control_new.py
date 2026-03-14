#!/usr/bin/env python3

# ============================================================
# CONTROL.py
# Logika kontrol drone: centering & payload drop
# Versi pymavlink — terintegrasi dengan shared_state.py
#
# ALUR DATA:
#   mavlink_thread  →  shared_state  ←  target_detection_25
#         ↑                 ↕
#         └──────────  CONTROL  (baca + tulis shared_state)
# ============================================================

import logging
import time
import threading
import math
import numpy as np
from simple_pid import PID
from haversine import haversine, Unit
from pymavlink import mavutil
from fall_physics import calc_horizontal_travel_dist
import shared_state
from wp_config import (
    APPROACH_WP_INDEX_1, APPROACH_WP_INDEX_2,
    DROP_ZONE_WP_1, DROP_ZONE_WP_2,
    DROP_ZONE_ELEVATION,
    APPROACH, DROPPING, APPROACH_TO_DROP_MAP
)

# ============================================================
# KONFIGURASI
# ============================================================

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

PORT      = '/dev/ttyACM0'
BAUDRATE  = 115200
LOOP_RATE = 30  # Hz

# ============================================================
# PARAMETER KONTROL
# ============================================================

RUDDER_CHANNEL_INDEX         = 4  # Channel RC rudder
DROP_MECHANISM_CHANNEL_INDEX = 7  # Channel RC servo payload
ACCEPTABLE_DEVIATION         = 0.05

DROP_MECHANISM_PWM = {
    DROP_ZONE_WP_1: 1495,
    DROP_ZONE_WP_2: 970,
}

yaw_pid = PID(Kp=12, Ki=0, Kd=0.11, setpoint=0, output_limits=(-400, 400))

master = None

# ============================================================
# FUNGSI HELPER
# ============================================================

def coordinate_angle(lat1, lon1, lat2, lon2):
    """Hitung bearing antara dua koordinat (derajat, 0-360)"""
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dLon    = lon2 - lon1
    y       = math.sin(dLon) * math.cos(lat2)
    x       = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360


def set_override_pwm(rudder_pwm, dropping_wp_index):
    """Kirim RC override untuk rudder saja"""
    global master

    if rudder_pwm == 0:
        return

    rc_vals = [65535] * 8  # 65535 = abaikan channel ini
    if 1 <= RUDDER_CHANNEL_INDEX <= 8:
        rc_vals[RUDDER_CHANNEL_INDEX - 1] = int(rudder_pwm)
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            *rc_vals
        )
    with shared_state.lock:
        shared_state.rudder_out_pwm = int(rudder_pwm)


def trigger_drop(dropping_wp_index):
    """Trigger pelepasan payload (kondisi normal)"""
    global master
    with shared_state.lock:
        shared_state.is_payload_dropped = True
        shared_state.centering_process  = False

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
    with shared_state.lock:
        shared_state.payload_out_pwm = target_pwm
    logging.info(f'===== PAYLOAD RELEASED (TRIGGER) | PWM: {target_pwm} =====')


def failsafe_drop(dropping_wp_index):
    """Trigger pelepasan payload (failsafe)"""
    global master
    with shared_state.lock:
        shared_state.is_payload_dropped = True
        shared_state.centering_process  = False

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
    with shared_state.lock:
        shared_state.payload_out_pwm = target_pwm
    logging.info(f'===== PAYLOAD RELEASED (FAILSAFE) | PWM: {target_pwm} =====')


# ============================================================
# THREAD: BACA DATA MAVLINK
# ============================================================

def mavlink_data_update():
    global master
    logging.info('[MAVLINK] Thread started')

    try:
        master = mavutil.mavlink_connection(PORT, baud=BAUDRATE)
        master.wait_heartbeat()
        logging.info('[MAVLINK] Heartbeat received — connected!')
        with shared_state.lock:
            shared_state.fcu_connected = True
    except Exception as e:
        logging.error(f'[MAVLINK] Connection error: {e}')
        return

    while True:
        with shared_state.lock:
            still_running = shared_state.running
        if not still_running:
            break

        msg = master.recv_match(
            type=['HEARTBEAT', 'MISSION_ITEM_REACHED', 'VFR_HUD',
                  'GLOBAL_POSITION_INT', 'ATTITUDE', 'SERVO_OUTPUT_RAW'],
            blocking=False
        )

        if not msg:
            time.sleep(0.01)
            continue

        msg_type = msg.get_type()

        with shared_state.lock:

            if msg_type == 'HEARTBEAT':
                shared_state.fcu_mode      = master.flightmode
                shared_state.is_armed      = master.motors_armed()
                shared_state.fcu_connected = True

            elif msg_type == 'MISSION_ITEM_REACHED':
                shared_state.wp_reached = msg.seq
                logging.info(f'[MAVLINK] WP REACHED: {msg.seq}')

            elif msg_type == 'GLOBAL_POSITION_INT':
                shared_state.current_latitude  = msg.lat / 1e7
                shared_state.current_longitude = msg.lon / 1e7
                shared_state.alt               = msg.relative_alt / 1000.0
                if msg.hdg != 65535:
                    shared_state.current_heading = msg.hdg / 100.0

            elif msg_type == 'VFR_HUD':
                shared_state.groundspeed         = msg.groundspeed
                shared_state.airspeed            = msg.airspeed
                shared_state.throttle_percentage = msg.throttle

            elif msg_type == 'ATTITUDE':
                shared_state.pitch = math.degrees(msg.pitch)

            elif msg_type == 'SERVO_OUTPUT_RAW':
                shared_state.rudder_out_pwm  = msg.servo4_raw
                shared_state.payload_out_pwm = msg.servo7_raw

        time.sleep(0.02)

    logging.info('[MAVLINK] Thread stopped')


# ============================================================
# MAIN LOOP — LOGIKA KONTROL
# ============================================================

def main():
    logging.info('===== CONTROL START =====')

    mavlink_thread = threading.Thread(target=mavlink_data_update, daemon=True)
    mavlink_thread.start()

    logging.info('Waiting for FC connection...')
    while True:
        with shared_state.lock:
            connected = shared_state.fcu_connected
        if connected:
            break
        time.sleep(1)

    logging.info('SYSTEM READY. Waiting for Approach WP...')

    current_drop_target  = -1
    centering_loop_count = 0

    try:
        while True:
            loop_start = time.time()

            with shared_state.lock:
                wp_passed   = shared_state.wp_reached
                lat         = shared_state.current_latitude
                lon         = shared_state.current_longitude
                alt         = shared_state.alt
                gs          = shared_state.groundspeed
                ias         = shared_state.airspeed
                mode        = shared_state.fcu_mode
                detected    = shared_state.is_target_detected
                tx          = shared_state.target_x
                dropped     = shared_state.is_payload_dropped
                centering   = shared_state.centering_process

            # ------------------------------------------------
            # FASE 1: RESET FLAG (sebelum approach)
            # ------------------------------------------------
            if (wp_passed + 1) in APPROACH:
                if dropped:
                    logging.info('[CONTROL] Resetting drop flag for next drop zone')
                    with shared_state.lock:
                        shared_state.is_payload_dropped = False
                        shared_state.is_target_aligned  = False
                        shared_state.centering_process  = False
                    centering_loop_count = 0

            # ------------------------------------------------
            # FASE 2: APPROACH & DROPPING
            # ------------------------------------------------
            if wp_passed in APPROACH and mode == 'AUTO':

                # Reset counter jika ini adalah approach WP baru
                if current_drop_target != APPROACH_TO_DROP_MAP[wp_passed]:
                    centering_loop_count = 0

                with shared_state.lock:
                    shared_state.centering_process = True

                centering_loop_count += 1
                current_drop_target = APPROACH_TO_DROP_MAP[wp_passed]

                elevation_ref    = DROP_ZONE_ELEVATION.get(current_drop_target, 0.0)
                h_effective      = max(0.1, alt - elevation_ref)
                drop_travel_dist = calc_horizontal_travel_dist(gs, h_effective, ias)

                current_loc         = (lat, lon)
                target_loc          = tuple(DROPPING[current_drop_target])
                coordinate_distance = haversine(current_loc, target_loc, unit=Unit.METERS)

                with shared_state.lock:
                    shared_state.drop_travel_dist = drop_travel_dist
                    shared_state.coord_dist       = coordinate_distance

                logging.info(f'[CONTROL] Drop Travel: {drop_travel_dist:.2f}m | Coord Dist: {coordinate_distance:.2f}m')

                # ----------------------------------------
                # GUIDANCE: Centering ke target
                # ----------------------------------------
                rudder_pwm = 1500

                if detected:
                    deviation  = tx - 0.5
                    rudder_pwm = 1500 + int(yaw_pid(deviation))
                    logging.info(f'[CONTROL] VISION | Dev: {deviation:.3f} | PWM: {rudder_pwm}')

                    aligned = abs(deviation) < ACCEPTABLE_DEVIATION
                    with shared_state.lock:
                        shared_state.is_target_aligned = aligned

                    if aligned:
                        logging.info('[CONTROL] Target ALIGNED')

                    set_override_pwm(rudder_pwm, current_drop_target)

                else:
                    approach_coord = APPROACH[wp_passed]
                    drop_coord     = DROPPING[current_drop_target]

                    track_angle = coordinate_angle(
                        approach_coord[0], approach_coord[1],
                        drop_coord[0],     drop_coord[1]
                    )
                    plane_angle = coordinate_angle(
                        lat, lon,
                        drop_coord[0], drop_coord[1]
                    )

                    deviation_angle = (track_angle - plane_angle + 180) % 360 - 180
                    error_norm      = np.clip(deviation_angle / 45.0, -1.0, 1.0)
                    rudder_pwm      = 1500 + int(yaw_pid(error_norm))

                    logging.info(f'[CONTROL] GPS | Dev Angle: {deviation_angle:.2f} | PWM: {rudder_pwm}')

                    aligned = abs(deviation_angle) < ACCEPTABLE_DEVIATION and centering
                    with shared_state.lock:
                        shared_state.is_target_aligned = aligned

                    set_override_pwm(rudder_pwm, current_drop_target)

                # ----------------------------------------
                # DROPPING
                # ----------------------------------------
                drop_allowed = centering_loop_count >= 3

                if coordinate_distance <= drop_travel_dist and not dropped and drop_allowed:
                    logging.warning(f'[CONTROL] DROP! Dist {coordinate_distance:.2f} <= {drop_travel_dist:.2f} | Loops: {centering_loop_count}')
                    trigger_drop(current_drop_target)
                elif not drop_allowed:
                    logging.info(f'[CONTROL] Waiting centering... loop {centering_loop_count}/3')

            # ------------------------------------------------
            # FASE 3: FAILSAFE
            # ------------------------------------------------
            elif wp_passed in DROPPING and not dropped:
                if wp_passed == current_drop_target:
                    logging.warning(f'[CONTROL] FAILSAFE at WP {wp_passed}')
                    failsafe_drop(current_drop_target)
                    current_drop_target  = -1
                    centering_loop_count = 0

            # ------------------------------------------------
            # Loop rate limiter
            # ------------------------------------------------
            elapsed    = time.time() - loop_start
            sleep_time = (1.0 / LOOP_RATE) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        logging.info('[CONTROL] Stopped by user')
        with shared_state.lock:
            shared_state.running = False
        mavlink_thread.join(timeout=2.0)
        logging.info('[CONTROL] MAVLink thread stopped')

    logging.info('===== CONTROL SHUTDOWN =====')


if __name__ == '__main__':
    main()