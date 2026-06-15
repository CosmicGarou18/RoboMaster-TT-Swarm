from djitellopySim import Tello
import time

drone = Tello()
drone.connect()
print(f"Battery: {drone.get_battery()}%")

drone.takeoff()
drone.move_forward(100)
time.sleep(1)
drone.rotate_clockwise(90)
time.sleep(1)
drone.land()