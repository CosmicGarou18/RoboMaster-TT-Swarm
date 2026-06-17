from djitellopy import Tello
import time

drone = Tello()
drone.connect()

while True:
    # Gets absolute distance to the floor directly beneath it
    height_cm = drone.get_distance_tof() 
    print(f"Height off ground: {height_cm} cm")
    time.sleep(0.5)