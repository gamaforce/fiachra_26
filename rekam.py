import os
import logging
import cv2
from datetime import datetime

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(message)s")

cap = cv2.VideoCapture(0)

frame_count = 0
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

if fps == 0:
    fps = 30

base_folder = "/home/julian/Documents/PlatformIO/Projects/Tracker"
video_folder = os.path.join(base_folder, "video")
frame_folder = os.path.join(base_folder, "frame")

os.makedirs(video_folder, exist_ok=True)
os.makedirs(frame_folder, exist_ok=True)

curr_time = datetime.now().strftime("%H_%d_%m_%Y")

fourcc = cv2.VideoWriter_fourcc(*"mp4v")
filename = os.path.join(video_folder, f"video_sampling_{curr_time}.mp4")
out = cv2.VideoWriter(filename, fourcc, fps, (frame_width, frame_height))

logging.info(f"Mulai merekam {filename}, dan disimpan di {video_folder}")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            logging.error("Port invalid")
            break

        out.write(frame)

        frame_name = os.path.join(
            frame_folder, f"gambar_{curr_time}_{frame_count}.jpg"
        )
        cv2.imshow("frame", frame)
        cv2.imwrite(frame_name, frame)
        frame_count += 1

        if cv2.waitKey(1) == ord("q"):
            logging.info("Disrupt by User")
            break

except Exception as e:
    logging.error(f"ERROR Unexpected {e}")

finally:
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    logging.info("Resource telah ditutup")
