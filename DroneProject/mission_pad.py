from djitellopy import Tello
import time
import math

tello = Tello()
tello.connect()
print(f"Battery: {tello.get_battery()}%")

# Enable detection
tello.send_command_with_return("mon")
tello.send_command_with_return("mdirection 0")

# Take off to 80cm (good detection height)
print("\n🚁 Taking off to 80cm...")
tello.takeoff()
time.sleep(1)
tello.move_up(80)
time.sleep(2)

# Search for pad (drone may not be directly above it)
print("\n🔍 Searching for mission pad...")

pad_found = False
search_radius = 50  # cm
positions = [
    (0, 0),      # Center
    (30, 0),     # Right
    (-30, 0),    # Left
    (0, 30),     # Forward
    (0, -30),    # Back
    (30, 30),    # Diagonal
    (-30, -30),
]

for x_pos, y_pos in positions:
    print(f"   Checking position: X={x_pos}, Y={y_pos}")
    
    # Move to search position
    tello.send_command_with_return(f"go {x_pos} {y_pos} 0 50 m-1")
    time.sleep(2)
    
    # Check for pad
    for attempt in range(3):
        status = tello.get_current_state()
        pad_id = status.get('mid', -1)
        if pad_id >= 0:
            print(f"✅ Pad {pad_id} found at X={x_pos}, Y={y_pos}!")
            pad_found = True
            break
        time.sleep(0.5)
    
    if pad_found:
        break

if not pad_found:
    print("❌ Pad not found in search area!")
    tello.land()
    tello.end()
    exit()

# Center and land
print("\n🎯 Centering over pad...")
for attempt in range(8):
    status = tello.get_current_state()
    x = status.get('x', 0)
    y = status.get('y', 0)
    distance = math.sqrt(x*x + y*y)
    
    if distance < 10:
        print("✅ Centered!")
        break
    
    move_x = max(-30, min(30, int(x * 0.5)))
    move_y = max(-30, min(30, int(y * 0.5)))
    tello.send_rc_control(move_x, move_y, 0, 0)
    time.sleep(0.5)

tello.send_rc_control(0, 0, 0, 0)

# Descend and land
print("\n⬇️ Descending to land...")
tello.move_down(60)
time.sleep(2)

print("\n🛬 Final landing...")
tello.land()

tello.send_command_with_return("moff")
tello.end()
print("\n✅ Mission complete!")