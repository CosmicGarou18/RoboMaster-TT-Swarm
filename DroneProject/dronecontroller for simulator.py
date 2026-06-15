import socket
import time

# ─── CHANGE THIS to Laptop B's actual IP address ───
DRONE_IP = "10.138.240.20"   # <-- Find this on Laptop B using 'ipconfig'
DRONE_PORT = 8889

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(3)

def send_command(command, wait=0.3):
    print(f"Sending: {command}")
    sock.sendto(command.encode('utf-8'), (DRONE_IP, DRONE_PORT))
    time.sleep(wait)
    
    # Try to get response (optional)
    try:
        data, _ = sock.recvfrom(1024)
        print(f"   → {data.decode('utf-8')}")
    except socket.timeout:
        pass

# ─── YOUR FLIGHT MISSION ─────────────────────────────
print("\n🚀 Starting mission...\n")

# Enter SDK mode
send_command("command", wait=1)

# Take off
send_command("takeoff", wait=2)

# Fly a square
send_command("forward 100", wait=2)
send_command("cw 90", wait=1)
send_command("forward 100", wait=2)
send_command("cw 90", wait=1)
send_command("forward 100", wait=2)
send_command("cw 90", wait=1)
send_command("forward 100", wait=2)

# Land
send_command("land", wait=2)

# Reset the simulator for next mission (optional)
send_command("reset", wait=2)

print("\n✅ Mission complete!")