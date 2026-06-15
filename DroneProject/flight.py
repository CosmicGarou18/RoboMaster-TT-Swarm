from djitellopy import Tello
import time

# Drone IPs
DRONE1_IP = "10.65.164.114"
DRONE2_IP = "10.65.164.76"

# Initialize
drone1 = Tello(DRONE1_IP)
drone2 = Tello(DRONE2_IP)

drone1.connect()
drone2.connect()

print(f"Battery: D1={drone1.get_battery()}% | D2={drone2.get_battery()}%")

# Take off
drone1.takeoff()
drone2.takeoff()
time.sleep(2)

# ========== PART 1: SYNCHRONIZED SPINS ==========
print("🎡 Sync spins...")
for _ in range(3):
    drone1.send_rc_control(0, 0, 0, 100)   # Spin right fast
    drone2.send_rc_control(0, 0, 0, -100)  # Spin left fast
    time.sleep(0.8)
    drone1.send_rc_control(0, 0, 0, 0)
    drone2.send_rc_control(0, 0, 0, 0)
    time.sleep(0.3)
    drone1.send_rc_control(0, 0, 0, -100)  # Reverse
    drone2.send_rc_control(0, 0, 0, 100)
    time.sleep(0.8)
    drone1.send_rc_control(0, 0, 0, 0)
    drone2.send_rc_control(0, 0, 0, 0)
    time.sleep(0.5)

# ========== PART 2: FLIP SEQUENCE (one after another) ==========
print("🤸 Flip sequence...")
time.sleep(1)
drone1.flip_forward()
time.sleep(2)
drone2.flip_forward()
time.sleep(2)
drone1.flip_back()
time.sleep(2)
drone2.flip_back()
time.sleep(2)

# ========== PART 3: SHAKE / WOBBLE (small fast movements) ==========
print("🎛️ Wobble effect...")
for _ in range(4):
    drone1.send_rc_control(30, 0, 0, 0)    # Quick right
    drone2.send_rc_control(-30, 0, 0, 0)   # Quick left
    time.sleep(0.2)
    drone1.send_rc_control(-30, 0, 0, 0)   # Quick left
    drone2.send_rc_control(30, 0, 0, 0)    # Quick right
    time.sleep(0.2)
drone1.send_rc_control(0, 0, 0, 0)
drone2.send_rc_control(0, 0, 0, 0)
time.sleep(0.5)

# ========== PART 4: UP-DOWN BOUNCE ==========
print("🏀 Bouncing...")
for _ in range(3):
    drone1.send_rc_control(0, 0, 40, 0)
    drone2.send_rc_control(0, 0, 40, 0)
    time.sleep(0.4)
    drone1.send_rc_control(0, 0, -40, 0)
    drone2.send_rc_control(0, 0, -40, 0)
    time.sleep(0.4)
drone1.send_rc_control(0, 0, 0, 0)
drone2.send_rc_control(0, 0, 0, 0)

# ========== PART 5: SIDE-TO-SIDE SWAY ==========
print("🌊 Swaying...")
for _ in range(3):
    drone1.send_rc_control(50, 0, 0, 0)
    drone2.send_rc_control(-50, 0, 0, 0)
    time.sleep(0.6)
    drone1.send_rc_control(-50, 0, 0, 0)
    drone2.send_rc_control(50, 0, 0, 0)
    time.sleep(0.6)
drone1.send_rc_control(0, 0, 0, 0)
drone2.send_rc_control(0, 0, 0, 0)

# ========== PART 6: FINAL FAST SPINS + FLIP ==========
print("🎉 Grand finale...")
for _ in range(2):
    drone1.send_rc_control(0, 0, 0, 200)
    drone2.send_rc_control(0, 0, 0, -200)
    time.sleep(0.5)
    drone1.send_rc_control(0, 0, 0, -200)
    drone2.send_rc_control(0, 0, 0, 200)
    time.sleep(0.5)
drone1.send_rc_control(0, 0, 0, 0)
drone2.send_rc_control(0, 0, 0, 0)
time.sleep(1)

drone1.flip_forward()
time.sleep(1.5)
drone2.flip_forward()

# ========== LAND ==========
print("🛬 Landing...")
drone1.land()
drone2.land()

print("✅ Lightshow complete!")