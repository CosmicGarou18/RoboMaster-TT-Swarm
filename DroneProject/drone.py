from djitellopySim import Tello
import socket
import time

# ─── UDP Server (listens for commands) ───
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 8889))
print("🚁 Drone simulator LISTENING on port 8889...")
print("   Press Ctrl+C to quit.\n")

# ─── Create the drone ───
drone = Tello()
drone.connect()
print("✅ Simulator ready.\n")

def send_response(addr, message="ok"):
    sock.sendto(message.encode('utf-8'), addr)

def reset_drone():
    """Reset drone to fresh state without restarting the script"""
    global drone
    print("   🔄 Resetting drone...")
    try:
        drone.land()
    except:
        pass
    try:
        drone.end()
    except:
        pass
    time.sleep(1)
    drone = Tello()
    drone.connect()
    print("   ✅ Drone reset. Ready for next mission.\n")

print("─────────────────────────────────────────")
print("Waiting for commands...\n")

while True:
    try:
        data, addr = sock.recvfrom(1024)
        command = data.decode('utf-8').strip()
        print(f"📡 Received: '{command}' from {addr}")
        
        if command == "command":
            send_response(addr)
        
        elif command == "takeoff":
            drone.takeoff()
            send_response(addr)
        
        elif command == "land":
            drone.land()
            send_response(addr)
        
        elif command == "reset":
            reset_drone()
            send_response(addr)
        
        elif command.startswith("forward"):
            dist = int(command.split(" ")[1])
            drone.move_forward(dist)
            send_response(addr)
        
        elif command.startswith("back"):
            dist = int(command.split(" ")[1])
            drone.move_back(dist)
            send_response(addr)
        
        elif command.startswith("left"):
            dist = int(command.split(" ")[1])
            drone.move_left(dist)
            send_response(addr)
        
        elif command.startswith("right"):
            dist = int(command.split(" ")[1])
            drone.move_right(dist)
            send_response(addr)
        
        elif command.startswith("cw"):
            angle = int(command.split(" ")[1])
            drone.rotate_clockwise(angle)
            send_response(addr)
        
        elif command.startswith("ccw"):
            angle = int(command.split(" ")[1])
            drone.rotate_counter_clockwise(angle)
            send_response(addr)
        
        elif command == "battery?":
            send_response(addr, "87")
        
        else:
            send_response(addr, "error")
            print(f"   ⚠️ Unknown command")
    
    except Exception as e:
        print(f"   ⚠️ Error: {e}")
        continue