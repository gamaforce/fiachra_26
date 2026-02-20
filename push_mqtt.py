#!/usr/bin/env python3

import time 
import paho.mqtt.client as mqtt
import json
import random
import logging
from datetime import datetime
from pymavlink import mavutil

TOPIC = "julian/message"
BROKER = "broker.emqx.io"
PORT = 1883
CLIENT_ID = f"tracker-{random.randint(0,1000)}"

client = mqtt.Client()
client.connect(BROKER, PORT, 60)
client.loop_start()

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(message)s")

def gps_update(master):
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
    if msg :
        current_alt = msg.relative_alt/1000
        current_lat = msg.lat/1e7
        current_lon = msg.lon/1e7
    
    payload = {
        "device" : CLIENT_ID,
        "alt" : current_alt,
        "lat" : current_lat,
        "lon" : current_lon
    }

    json_message= json.dumps(payload)
    client.publish(TOPIC, json_message)
    logging.info(f"mengirim {json_message}")

alt_lidar = 0
def get_lidar():
    global alt_lidar
    if master is None :
        return
    
    msg = master.recv_match(type="DISTANCE_SENSOR", blocking=False)
    if msg :
        logging.info("Cek orientasi")
        if msg.orientation==25 or msg.orientation ==0:
            raw_dist = msg.current_distance
            if 0 < raw_dist < 65535 :
                alt_lidar = raw_dist
                alt_lidar_m = alt_lidar/100
                logging.info(f"Ketinggian alt lidar {alt_lidar_m} m")
            else :
                pass



try :
    port_conn = "/dev/ttyACM0"
    master = mavutil.mavlink_connection(port_conn, baud=115200)
    logging.info("waiting for heartbeat!")
    master.wait_heartbeat()
    logging.info("HEARTBEAT RECEIVED! System Active")

    while True :
        gps_update(master)
        time.sleep(0.5)

except Exception as e :
    logging.error(f" Error unexpected {e}")


