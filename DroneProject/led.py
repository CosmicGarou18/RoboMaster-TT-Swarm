from djitellopy import Tello
import time

tello = Tello('192.168.10.1')
tello.connect()
print(f"Battery: {tello.get_battery()}%")

# 1. Display heart using the correct command (from your manual page 7)
# EXT mled s r/b/p xxxx - where xxxx can be "heart"
print("Displaying heart...")
response = tello.send_command_with_return("EXT mled s r heart")
print(f"Response: {response}")
time.sleep(3)

# 2. Clear/off display
print("Turning off...")
response = tello.send_command_with_return("EXT mled s p 0")
print(f"Response: {response}")
time.sleep(1)

# 3. Display letter 'A' in red
print("Displaying 'A'...")
response = tello.send_command_with_return("EXT mled s r A")
print(f"Response: {response}")
time.sleep(3)

# 4. Scroll text "HI" to the left (l) in red (r) at 1Hz
print("Scrolling 'HI'...")
response = tello.send_command_with_return("EXT mled l r 1 HI")
print(f"Response: {response}")
time.sleep(4)

# 5. Top LED - solid red (r=255, g=0, b=0)
print("Top LED solid red...")
response = tello.send_command_with_return("EXT led 255 0 0")
print(f"Response: {response}")
time.sleep(2)

# 6. Top LED - blinking red/green
print("Top LED blinking...")
response = tello.send_command_with_return("EXT led bl 1 255 0 0 0 255 0")
print(f"Response: {response}")
time.sleep(4)

# 7. Turn off everything
print("All off...")
tello.send_command_with_return("EXT mled s p 0")
tello.send_command_with_return("EXT led 0 0 0")

tello.end()
print("Done!")