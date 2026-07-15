"""
Downward camera viewer using djitellopy's Tello class for command/response
handling, ffplay (external process) for video display.

Requires a WORKING opencv install (djitellopy imports cv2 at module load,
even though we don't use cv2's video capture/display here).
Requires ffmpeg on PATH (`ffplay -version` to check).
Requires firmware v02.05.01.17 / SDK 3.0 for `downvision`.
"""
import subprocess
import time
from djitellopy import Tello

VIDEO_PORT = 11111

tello = Tello()
tello.connect()

sdk_version = tello.query_sdk_version()
print(f"SDK version: {sdk_version}")
if sdk_version != "30":
    print("downvision requires SDK 3.0 (firmware v02.05.01.17). "
          "Update via the Tello app (remove Expansion Kit first) before this will work.")

print(f"Battery: {tello.get_battery()}%")

tello.streamon()
time.sleep(0.5)
reply = tello.set_video_direction(tello.CAMERA_DOWNWARD)
print(f"downvision 1 -> {reply}")

print("Launching ffplay...")
ffplay = subprocess.Popen([
    "ffplay",
    "-i", f"udp://0.0.0.0:{VIDEO_PORT}",
    "-fflags", "nobuffer",
    "-flags", "low_delay",
    "-framedrop",
    "-window_title", "Tello Downward Camera",
])

try:
    ffplay.wait()
except KeyboardInterrupt:
    ffplay.terminate()
finally:
    tello.set_video_direction(tello.CAMERA_FORWARD)
    tello.streamoff()