"""
Velocity_Finder.py (forward_obstacle_stop, thread-safe version)
Fly forward at fixed rc, z-axis held by PID via TOF, front-TOF (via EXT tof?)
triggers brake+land. Prints time-of-flight.

FIX: all djitellopy calls now serialized through a shared lock — djitellopy
is NOT thread-safe, and the front-TOF polling thread was stepping on
command/response pairs on the main control thread (e.g. 'takeoff' reading
back a stray 'tof 8191' response). Every single call site that touches
`tello` must go through safe_call().
"""

import time
import threading
from djitellopy import Tello

# ---------------- CONFIG ----------------
FORWARD_RC            = 30
BRAKE_RC              = -100
BRAKE_DURATION_S      = 0.3    # ASSUMPTION — tune this
OBSTACLE_THRESHOLD_MM = 500    # ASSUMPTION — you never gave me a real number, change this

ORIGIN_ALTITUDE_CM = 100
Z_KP, Z_KI, Z_KD   = 5.0, 0.333, 1.625

CONTROL_HZ         = 20
SETTLE_S           = 3.0
SATURATION_ABORT_S = 1.5
BATTERY_ABORT_PCT  = 15
HEARTBEAT_S        = 5.0
FRONT_TOF_POLL_S   = 0.15

OUTPUT_CSV = "output/forward_obstacle_flight_log.csv"

# ---------------- GLOBAL LOCK — every djitellopy call goes through this ----------------
tello_lock = threading.Lock()

def safe_call(fn, *args, **kwargs):
    """Serializes ALL access to the Tello command/response channel.
    djitellopy has no internal locking — two threads calling it at once
    WILL cross-wire responses (this is exactly what broke takeoff)."""
    with tello_lock:
        return fn(*args, **kwargs)


# ---------------- PID CLASS ----------------
class PID:
    def __init__(self, kp, ki, kd, output_limit=100):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_limit = output_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.lock = threading.Lock()

    def reset(self):
        with self.lock:
            self.integral = 0.0
            self.prev_error = 0.0

    def compute(self, error, dt):
        with self.lock:
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
            self.prev_error = error
            output = self.kp * error + self.ki * self.integral + self.kd * derivative
            return max(-self.output_limit, min(self.output_limit, output))


# ---------------- HEIGHT READER ----------------
class HeightReader:
    def __init__(self, tello):
        self.tello = tello
        self.last_good = ORIGIN_ALTITUDE_CM
        self.lock = threading.Lock()

    def read(self):
        try:
            raw = safe_call(self.tello.get_distance_tof)
        except Exception:
            return self.last_good
        if raw == 8888 or raw < 1 or raw > 800:
            return self.last_good
        with self.lock:
            if abs(raw - self.last_good) > 50:
                return self.last_good
            self.last_good = raw
        return raw


# ---------------- SAFE READS ----------------
def safe_int_battery(tello, last_good=[100]):
    try:
        val = int(safe_call(tello.get_battery))
        last_good[0] = val
        return val
    except Exception:
        return last_good[0]


# ---------------- FRONT TOF THREAD ----------------
class FrontTOFThread(threading.Thread):
    def __init__(self, tello):
        super().__init__(daemon=True)
        self.tello = tello
        self.lock = threading.Lock()
        self.distance_mm = None
        self.running = True

    def run(self):
        while self.running:
            try:
                resp = safe_call(self.tello.send_command_with_return, "EXT tof?")
                resp = resp.strip()
                if "keepalive" in resp:
                    time.sleep(FRONT_TOF_POLL_S)
                    continue
                if resp.startswith("tof"):
                    val = int(resp.split()[1])
                    if 0 <= val <= 8192:
                        with self.lock:
                            self.distance_mm = val
            except Exception:
                pass
            time.sleep(FRONT_TOF_POLL_S)

    def get_distance(self):
        with self.lock:
            return self.distance_mm

    def stop(self):
        self.running = False


# ---------------- SAFETY KEYS ----------------
class SafetyKeys(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.land_requested = False
        self.emergency_requested = False
        self.running = True

    def run(self):
        try:
            import keyboard
        except ImportError:
            print("[WARN] 'keyboard' module not installed — manual L/E override disabled.")
            return
        while self.running:
            if keyboard.is_pressed('l'):
                self.land_requested = True
            if keyboard.is_pressed('e'):
                self.emergency_requested = True
            time.sleep(0.05)

    def stop(self):
        self.running = False


# ---------------- MAIN ----------------
def main():
    import os
    os.makedirs("output", exist_ok=True)

    tello = Tello()
    safe_call(tello.connect)

    bat = safe_int_battery(tello)
    print(f"[INFO] Battery: {bat}%")
    if bat < BATTERY_ABORT_PCT:
        print("[ABORT] Battery too low to fly.")
        return

    height_reader = HeightReader(tello)
    z_pid = PID(Z_KP, Z_KI, Z_KD, output_limit=60)
    front_tof = FrontTOFThread(tello)
    front_tof.start()
    safety = SafetyKeys()
    safety.start()

    obstacle_triggered = False
    saturation_start = None
    last_heartbeat = time.time()
    t_start = None
    t_end = None
    z_output = 0

    def send_rc(pitch):
        safe_call(tello.send_rc_control, 0, pitch, int(z_output), 0)

    try:
        safe_call(tello.takeoff)
        t_start = time.time()
        print(f"[INFO] Takeoff at t={t_start:.3f}")

        print(f"[INFO] Settling for {SETTLE_S}s before control starts...")
        time.sleep(SETTLE_S)

        z_pid.reset()
        last_tick = time.time()

        while True:
            now = time.time()
            dt = now - last_tick
            if dt <= 0:
                dt = 1.0 / CONTROL_HZ
            last_tick = now

            if safety.emergency_requested:
                print("[MANUAL] Emergency stop requested.")
                safe_call(tello.emergency)
                break
            if safety.land_requested:
                print("[MANUAL] Land requested.")
                safe_call(tello.send_rc_control, 0, 0, 0, 0)
                safe_call(tello.land)
                t_end = time.time()
                break

            bat = safe_int_battery(tello)
            if bat < BATTERY_ABORT_PCT:
                print(f"[SAFETY] Battery {bat}% below threshold — landing.")
                safe_call(tello.send_rc_control, 0, 0, 0, 0)
                safe_call(tello.land)
                t_end = time.time()
                break

            current_height = height_reader.read()
            z_error = ORIGIN_ALTITUDE_CM - current_height
            z_output = z_pid.compute(z_error, dt)

            if abs(z_output) >= 59:
                if saturation_start is None:
                    saturation_start = now
                elif now - saturation_start >= SATURATION_ABORT_S:
                    print("[SAFETY] z-axis output pinned — landing.")
                    safe_call(tello.send_rc_control, 0, 0, 0, 0)
                    safe_call(tello.land)
                    t_end = time.time()
                    break
            else:
                saturation_start = None

            front_dist = front_tof.get_distance()
            if front_dist is not None and front_dist < OBSTACLE_THRESHOLD_MM:
                print(f"[OBSTACLE] Front TOF={front_dist}mm < {OBSTACLE_THRESHOLD_MM}mm — braking.")
                safe_call(tello.send_rc_control, 0, BRAKE_RC, 0, 0)
                time.sleep(BRAKE_DURATION_S)
                safe_call(tello.send_rc_control, 0, 0, 0, 0)
                time.sleep(0.3)
                safe_call(tello.land)
                t_end = time.time()
                obstacle_triggered = True
                break

            send_rc(FORWARD_RC)

            if now - last_heartbeat >= HEARTBEAT_S:
                safe_call(tello.get_battery)
                last_heartbeat = now

            time.sleep(max(0, (1.0 / CONTROL_HZ) - (time.time() - now)))

    except KeyboardInterrupt:
        print("[Ctrl+C] Landing (not motor-cut).")
        try:
            safe_call(tello.send_rc_control, 0, 0, 0, 0)
            safe_call(tello.land)
        except Exception:
            pass
        t_end = time.time()

    finally:
        front_tof.stop()
        safety.stop()
        if t_start is not None:
            t_end = t_end or time.time()
            tof_flight = t_end - t_start
            print(f"[RESULT] Time of flight: {tof_flight:.2f}s "
                  f"({'obstacle-triggered' if obstacle_triggered else 'other exit'})")
            with open(OUTPUT_CSV, "a") as f:
                f.write(f"{t_start},{t_end},{tof_flight:.3f},{obstacle_triggered}\n")


if __name__ == "__main__":
    main()