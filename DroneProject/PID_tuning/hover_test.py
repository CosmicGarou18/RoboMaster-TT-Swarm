"""
Fixed-gain PID hover test — z + lateral simultaneously.

Gains (as given, UNVALIDATED — see warnings above):
  z:       Kp=6.25   Ki=0.333  Kd=1.625
  lateral: Kp=5.0    Ki=0.333  Kd=1.625   (same gains applied to both lr and fb)

Safety patterns carried over from prior scripts:
  - HeightReader: TOF-based z error only, NEVER get_height() (barometer/IMU fusion garbage indoors)
  - vgx/vgy single-integrated for lateral position error (linear drift, not quadratic)
  - safe_int_battery / safe_speed: never trust a raw SDK response as a number without checking
  - Saturation watchdog: pinned output >= 1.5s -> land
  - Battery watchdog: <15% -> land
  - Heartbeat: battery? every 5s
  - L = land, E = emergency cut, Ctrl+C = land (not raw motor cut)

Deps: pip install djitellopy keyboard
"""

import time
import threading
from djitellopy import Tello

# ---------------- Fixed gains (as specified) ----------------
Z_KP, Z_KI, Z_KD = 6.25, 0.333, 1.625
LAT_KP, LAT_KI, LAT_KD = 5.0, 0.333, 1.625

TARGET_HEIGHT_CM = 100
CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ
SATURATION_LIMIT = 40          # rc value cap, matches prior scripts' MAX_VCMD-style clamp
SATURATION_ABORT_S = 1.5
BATTERY_ABORT_PCT = 15
HEARTBEAT_S = 5

_last_good_battery = 100
_last_good_speed = {"x": 0.0, "y": 0.0}


def safe_int_battery(tello):
    global _last_good_battery
    try:
        v = int(tello.get_battery())
        _last_good_battery = v
        return v
    except (ValueError, TypeError):
        return _last_good_battery


def safe_speed(tello, axis):
    global _last_good_speed
    try:
        v = float(tello.get_speed_x() if axis == "x" else tello.get_speed_y())
        _last_good_speed[axis] = v
        return v
    except (ValueError, TypeError):
        return _last_good_speed[axis]


class HeightReader:
    """TOF-based height only. Rejects 8888 sentinel, out-of-range, >50cm jumps."""
    def __init__(self):
        self.last_good = TARGET_HEIGHT_CM

    def read(self, tello):
        try:
            d = tello.get_distance_tof()
        except Exception:
            return self.last_good
        if d == 8888 or d <= 1 or d >= 800:
            return self.last_good
        if abs(d - self.last_good) > 50:
            return self.last_good
        self.last_good = d
        return d


class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        return out


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class ControlLoopThread(threading.Thread):
    def __init__(self, tello):
        super().__init__(daemon=True)
        self.tello = tello
        self.running = True
        self.land_flag = False
        self.emergency_flag = False

        self.height_reader = HeightReader()
        self.pid_z = PID(Z_KP, Z_KI, Z_KD)
        self.pid_x = PID(LAT_KP, LAT_KI, LAT_KD)  # left/right
        self.pid_y = PID(LAT_KP, LAT_KI, LAT_KD)  # forward/back

        self.sat_timer = {"z": 0.0, "x": 0.0, "y": 0.0}
        self.last_heartbeat = time.time()
        self.status = {"height": TARGET_HEIGHT_CM, "err_z": 0, "err_x": 0, "err_y": 0,
                        "out_z": 0, "out_x": 0, "out_y": 0, "battery": 100, "flags": ""}

    def run(self):
        t = self.tello
        t.takeoff()
        time.sleep(3)  # post-takeoff settle — known race condition if you skip this
        self.pid_z.reset(); self.pid_x.reset(); self.pid_y.reset()

        while self.running:
            loop_start = time.time()

            if self.emergency_flag:
                t.emergency()
                break
            if self.land_flag:
                t.send_rc_control(0, 0, 0, 0)
                time.sleep(0.3)
                t.land()
                break

            battery = safe_int_battery(t)
            if battery < BATTERY_ABORT_PCT:
                self.status["flags"] = "BATTERY ABORT"
                self.land_flag = True
                continue

            height = self.height_reader.read(t)
            err_z = TARGET_HEIGHT_CM - height
            out_z = clamp(self.pid_z.step(err_z, DT), -SATURATION_LIMIT, SATURATION_LIMIT)

            vgx = safe_speed(t, "x")
            vgy = safe_speed(t, "y")
            # single-integrated position error, drift-unreliable past ~15s (unvalidated)
            err_x = -vgx
            err_y = -vgy
            out_x = clamp(self.pid_x.step(err_x, DT), -SATURATION_LIMIT, SATURATION_LIMIT)
            out_y = clamp(self.pid_y.step(err_y, DT), -SATURATION_LIMIT, SATURATION_LIMIT)

            # saturation watchdog per axis
            for axis, out in (("z", out_z), ("x", out_x), ("y", out_y)):
                if abs(out) >= SATURATION_LIMIT:
                    self.sat_timer[axis] += DT
                else:
                    self.sat_timer[axis] = 0.0
                if self.sat_timer[axis] >= SATURATION_ABORT_S:
                    self.status["flags"] = f"SATURATION ABORT ({axis})"
                    self.land_flag = True

            t.send_rc_control(int(out_x), int(out_y), int(out_z), 0)

            if time.time() - self.last_heartbeat >= HEARTBEAT_S:
                _ = safe_int_battery(t)
                self.last_heartbeat = time.time()

            self.status.update(height=height, err_z=err_z, err_x=err_x, err_y=err_y,
                                out_z=out_z, out_x=out_x, out_y=out_y, battery=battery)

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, DT - elapsed))


class SafetyKeys(threading.Thread):
    def __init__(self, loop):
        super().__init__(daemon=True)
        self.loop = loop

    def run(self):
        import keyboard
        while self.loop.running:
            if keyboard.is_pressed('l'):
                self.loop.land_flag = True
                time.sleep(0.5)
            if keyboard.is_pressed('e'):
                self.loop.emergency_flag = True
                time.sleep(0.5)
            time.sleep(0.05)


class DashboardThread(threading.Thread):
    def __init__(self, loop):
        super().__init__(daemon=True)
        self.loop = loop

    def run(self):
        while self.loop.running and self.loop.is_alive():
            s = self.loop.status
            print("\033c", end="")  # clear terminal
            print("=== PID HOVER (z + lateral, fixed gains) ===")
            print(f"z:  Kp={Z_KP} Ki={Z_KI} Kd={Z_KD}")
            print(f"lat:Kp={LAT_KP} Ki={LAT_KI} Kd={LAT_KD}")
            print("-" * 40)
            print(f"battery : {s['battery']}%")
            print(f"height  : {s['height']}cm  err_z={s['err_z']:.1f}  out_z={s['out_z']:.1f}")
            print(f"lateral : err_x={s['err_x']:.1f} out_x={s['out_x']:.1f} | "
                  f"err_y={s['err_y']:.1f} out_y={s['out_y']:.1f}")
            if s["flags"]:
                print(f">>> {s['flags']} <<<")
            print("-" * 40)
            print("L = land   E = emergency cut")
            time.sleep(0.1)


def main():
    tello = Tello()
    tello.connect()
    print(f"Battery: {tello.get_battery()}%")
    if tello.get_battery() < 30:
        print("Battery too low to safely run a hover test. Aborting.")
        return

    loop = ControlLoopThread(tello)
    keys = SafetyKeys(loop)
    dash = DashboardThread(loop)

    loop.start()
    keys.start()
    dash.start()

    try:
        while loop.is_alive():
            time.sleep(0.2)
    except KeyboardInterrupt:
        loop.land_flag = True
        time.sleep(3)

    loop.running = False
    print("Done.")


if __name__ == "__main__":
    main()