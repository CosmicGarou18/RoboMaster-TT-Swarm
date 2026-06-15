from djitellopy import Tello
import pygame
import cv2
import numpy as np
import math
import time

# ==========================
# CONFIG
# ==========================
DRONE_IP = "10.65.164.114"
SPEED = 60

# ==========================
# CONNECT
# ==========================
tello = Tello(DRONE_IP)

print("Connecting...")
tello.connect()

print("Battery:", tello.get_battery(), "%")

# ==========================
# PYGAME
# ==========================
pygame.init()
screen = pygame.display.set_mode((500, 300))
pygame.display.set_caption("Tello Controller")

font = pygame.font.SysFont("Arial", 22)

# ==========================
# POSITION ESTIMATION
# ==========================
x = 0.0
y = 0.0
z = 0.0

yaw_deg = 0.0

prev_time = time.time()

trajectory = [(0, 0)]

# ==========================
# MAP WINDOW
# ==========================
MAP_SIZE = 700
CENTER = MAP_SIZE // 2
SCALE = 60  # pixels per meter

running = True

print("""
CONTROLS

T = Takeoff
L = Land

W/S = Forward / Back
A/D = Left / Right

R/F = Up / Down

Q/E = Rotate Left / Right

X = Stop

ESC = Quit
""")

while running:

    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time

    lr = 0
    fb = 0
    ud = 0
    yaw = 0

    # --------------------------
    # EVENTS
    # --------------------------
    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:

            if event.key == pygame.K_t:
                tello.takeoff()

            elif event.key == pygame.K_l:
                tello.land()

            elif event.key == pygame.K_ESCAPE:
                running = False

            elif event.key == pygame.K_x:
                tello.send_rc_control(0, 0, 0, 0)

    keys = pygame.key.get_pressed()

    # --------------------------
    # CONTROL INPUT
    # --------------------------
    if keys[pygame.K_w]:
        fb = SPEED

    if keys[pygame.K_s]:
        fb = -SPEED

    if keys[pygame.K_a]:
        lr = -SPEED

    if keys[pygame.K_d]:
        lr = SPEED

    if keys[pygame.K_r]:
        ud = SPEED

    if keys[pygame.K_f]:
        ud = -SPEED

    if keys[pygame.K_q]:
        yaw = -SPEED

    if keys[pygame.K_e]:
        yaw = SPEED

    # --------------------------
    # SEND COMMAND
    # --------------------------
    tello.send_rc_control(lr, fb, ud, yaw)

    # --------------------------
    # POSITION ESTIMATION
    # --------------------------

    vx = fb / 100.0
    vy = lr / 100.0
    vz = ud / 100.0

    yaw_deg += yaw * 0.9 * dt

    yaw_rad = math.radians(yaw_deg)

    world_vx = (
        vx * math.cos(yaw_rad)
        - vy * math.sin(yaw_rad)
    )

    world_vy = (
        vx * math.sin(yaw_rad)
        + vy * math.cos(yaw_rad)
    )

    x += world_vx * dt
    y += world_vy * dt
    z += vz * dt

    trajectory.append((x, y))

    # --------------------------
    # PYGAME DISPLAY
    # --------------------------
    screen.fill((20, 20, 20))

    try:
        battery = tello.get_battery()
    except:
        battery = -1

    try:
        height = tello.get_height()
    except:
        height = -1

    lines = [
        f"Battery: {battery}%",
        f"Height: {height} cm",
        f"Yaw: {yaw_deg:.1f} deg",
        "",
        f"X: {x:.2f} m",
        f"Y: {y:.2f} m",
        f"Z: {z:.2f} m",
    ]

    y_text = 20

    for text in lines:
        surface = font.render(text, True, (255, 255, 255))
        screen.blit(surface, (20, y_text))
        y_text += 35

    pygame.display.update()

    # --------------------------
    # MAP
    # --------------------------
    map_img = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)

    cv2.line(map_img,
             (CENTER, 0),
             (CENTER, MAP_SIZE),
             (100, 100, 100), 1)

    cv2.line(map_img,
             (0, CENTER),
             (MAP_SIZE, CENTER),
             (100, 100, 100), 1)

    for i in range(1, len(trajectory)):

        x1, y1 = trajectory[i - 1]
        x2, y2 = trajectory[i]

        p1 = (
            int(CENTER + x1 * SCALE),
            int(CENTER - y1 * SCALE)
        )

        p2 = (
            int(CENTER + x2 * SCALE),
            int(CENTER - y2 * SCALE)
        )

        cv2.line(map_img, p1, p2, (0, 255, 0), 2)

    drone_pos = (
        int(CENTER + x * SCALE),
        int(CENTER - y * SCALE)
    )

    cv2.circle(map_img, drone_pos, 8, (0, 0, 255), -1)

    cv2.putText(
        map_img,
        f"({x:.1f},{y:.1f})",
        (drone_pos[0] + 10, drone_pos[1]),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2
    )

    cv2.imshow("Tello Position Map", map_img)

    if cv2.waitKey(1) & 0xFF == 27:
        running = False

    time.sleep(0.05)

# ==========================
# CLEANUP
# ==========================
try:
    tello.send_rc_control(0, 0, 0, 0)
except:
    pass

pygame.quit()
cv2.destroyAllWindows()