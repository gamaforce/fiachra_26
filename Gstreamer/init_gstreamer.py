#!/usr/bin/env python3

import gi
import logging
import sys

# Inisialisasi GStreamer
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Konfigurasi logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# Konfigurasi Stream
TARGET_IP = "192.168.1.100"  # IP Receiver
PORT = 5000
VIDEO_DEVICE = "/dev/video0"
WIDTH = 1280
HEIGHT = 720
FPS = 30
BITRATE = 3000  


class GStreamerSender:
    
    def __init__(self, target_ip=TARGET_IP, port=PORT):
        self.target_ip = target_ip
        self.port = port
        self.pipeline = None
        self.loop = None
        
        # Inisialisasi GStreamer
        Gst.init(None)
        
    def create_pipeline(self):
        # webcam
        #pipeline_str_web = "v4l2src device=/dev/video0 ! videoconvert ! appsink"

        # CSI Kamera
        # pipeline_str = (
        #     f"v4l2src device={VIDEO_DEVICE} ! "
        #     f"video/x-raw, width={WIDTH}, height={HEIGHT}, framerate={FPS}/1 ! "
        #     f"videoconvert ! "
        #     f"x26 "
        #     f"tune=zerolatency "
        #     f"bitrate={BITRATE} "
        #     f"speed-preset=ultrafast "
        #     f"key-int-max={FPS} ! "
        #     f"rtph264pay ! "
        #     f"udpsink host={self.target_ip} port={self.port}"
        # )

        pipeline_str = (
            f"libcamerasrc ! "
            f"video/x-raw,width={WIDTH},height={HEIGHT},framerate={FPS}/1 ! "
            f"queue max-size-buffers=1 leaky=downstream ! "
            f"videoconvert ! "
            f"v4l2h264enc bitrate={BITRATE*1000} ! "
            f"h264parse ! "
            f"rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={self.target_ip} port={self.port} sync=false async=false"
        )

        logging.info(f"Pipeline: {pipeline_str}")
        return Gst.parse_launch(pipeline_str)
    
    def on_message(self, bus, message):
        msg_type = message.type
        
        if msg_type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            logging.error(f"GStreamer Error: {err}")
            logging.debug(f"Debug info: {debug}")
            self.restart()
            
        elif msg_type == Gst.MessageType.EOS:
            logging.info("End of Stream (EOS)")
            self.stop()
            
        elif msg_type == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old_state, new_state, pending = message.parse_state_changed()
                logging.info(f"Pipeline state: {old_state.value_nick} → {new_state.value_nick}")
    
    def restart(self):
        self.stop()
        self.start()

    def start(self):
        try:
            logging.info("GStreamer Sender - 720p Low-Latency Streamer")
            logging.info(f"Target: {self.target_ip}:{self.port}")
            logging.info(f"Resolusi: {WIDTH}x{HEIGHT} @ {FPS}fps")
            logging.info(f"Bitrate: {BITRATE} kbps")
            logging.info(f"Codec: H.264 (tune=zerolatency)")
            
            # Buat dan jalankan pipeline
            self.pipeline = self.create_pipeline()
            
            # Setup bus untuk monitoring
            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect("message", self.on_message)
            
            # Mulai pipeline
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                logging.error("Gagal memulai pipeline")
                return False
            
            logging.info("Streaming dimulai! Tekan Ctrl+C untuk berhenti.")
            
            # Main loop
            self.loop = GLib.MainLoop()
            self.loop.run()
            
        except KeyboardInterrupt:
            logging.info("\nStreaming dihentikan oleh user")
        except Exception as e:
            logging.error(f"Error: {e}")
        finally:
            self.stop()
            
        return True
    
    def stop(self):
        logging.info("Menghentikan stream...")
        
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
            
        if self.loop:
            self.loop.quit()
            self.loop = None
            
        logging.info("Stream berhasil dihentikan")


def main():
    # Parse command line arguments
    if len(sys.argv) > 1:
        target_ip = sys.argv[1]
    else:
        target_ip = TARGET_IP
        
    if len(sys.argv) > 2:
        port = int(sys.argv[2])
    else:
        port = PORT
    
    # Buat dan jalankan sender
    sender = GStreamerSender(target_ip=target_ip, port=port)
    sender.start()


if __name__ == "__main__":
    main()
