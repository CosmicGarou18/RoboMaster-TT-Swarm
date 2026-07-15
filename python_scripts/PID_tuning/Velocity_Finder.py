"""
RC -> velocity characterization script (forward-only).

Flat sequence of (rc_value, trial_number) pairs, rc_value stepping by 10,
TRIALS_PER_RC trials each. For each trial:

  forward @ rc for MOVE_S sec, recording vgy
  wait for operator input:
    G = go        -> continue to the next trial (still airborne)
    L = land      -> land now, wait for repositioning, wait for next G,
                      takeoff, resume at the SAME next trial (nothing skipped)
  E = emergency motor cut, works at any point, any state

If landing happens mid-forward-segment (not just at the wait point), that
segment is aborted and discarded/retried — never recorded as a partial trial.

If the script itself lands due to battery <15% or z-axis PID saturation,
that's treated as a real fault: the whole test stops, it does NOT wait for
a G to resume.

Deps: pip install djitellopy keyboard
"""

import time
import threading
import statistics
import csv
import os
from djitellopy import Tello
from djitellopy.swarm import TelloSwarm

# ---------------- Config ----------------
RC_START, RC_END, RC_STEP = 10, 100, 10   # unconfirmed assumption, change if wrong
TRIALS_PER_RC = 10
MOVE_S = 4.0
POST_TAKEOFF_SETTLE_S = 3.0

Z_KP, Z_KI, Z_KD = 6.25, 0.333, 1.625
TARGET_HEIGHT_CM = 100

CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ
Z_SATURATION_LIMIT = 40
Z_SATURATION_ABORT_S = 5
BATTERY_ABORT_PCT = 15
HEARTBEAT_S = 5

OUTPUT_CSV = "output/rc_to_velocity.csv"

_last_good_battery = 100
_last_good_speed = {"x": 0.0, "y": 0.0 }


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
        return self.kp * error + self.ki * self.integral + self.kd * derivative


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class ControlLoopThread(threading.Thread):
    """z-hover PID runs continuously. fb externally settable via set_fb().
    Recording externally toggled; while recording, vgy samples collected."""

    def __init__(self, tello: Tello):
        super().__init__(daemon=True)
        self.tello = tello
        self.running = True
        self.land_flag = False
        self.emergency_flag = False
        self.abort_reason = ""   # "battery" / "z_saturation" / "" (manual/none)

        self.height_reader = HeightReader()
        self.pid_z = PID(Z_KP, Z_KI, Z_KD)

        self.lock = threading.Lock()
        self.fb = 0
        self.recording = False
        self.samples = []

        self.sat_timer_z = 0.0
        self.last_heartbeat = time.time()
        self.status = {"height": TARGET_HEIGHT_CM, "err_z": 0.0, "out_z": 0.0,
                        "battery": 100, "flags": ""}

    def set_fb(self, value):
        with self.lock:
            self.fb = value

    def start_recording(self):
        with self.lock:
            self.samples = []
            self.recording = True

    def stop_recording(self):
        with self.lock:
            self.recording = False
            return list(self.samples)

    def run(self):
        t: Tello = self.tello
        self.pid_z.reset()

        t0 = time.time()

        while True:
            loop_start = time.time()

            dist = self.tello.send_read_command_int("EXT tof?")
            if dist <= 100:
                t.send_rc_control(0, -100, 0, 0)
                time.sleep(0.5)
                t.send_rc_control(0, 0, 0, 0)
                time.sleep(0.3)
                t.land()

                dt = time.time() - t0
                vel = 6.6 / dt

                print(f"Time taken: {dt}")
                print(f"Velocity = {vel}")

                with open("output/out.csv", "+a") as f:
                    print(f"{fb}, {vel}", file=f)

                break

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
                self.abort_reason = "battery"
                self.land_flag = True
                continue

            height = self.height_reader.read(t)
            err_z = TARGET_HEIGHT_CM - height
            out_z = clamp(self.pid_z.step(err_z, DT), -Z_SATURATION_LIMIT, Z_SATURATION_LIMIT)

            if abs(out_z) >= Z_SATURATION_LIMIT:
                self.sat_timer_z += DT
            else:
                self.sat_timer_z = 0.0

            if self.sat_timer_z >= Z_SATURATION_ABORT_S:
                self.status["flags"] = "Z SATURATION ABORT"
                self.abort_reason = "z_saturation"
                self.land_flag = True

            with self.lock:
                fb = self.fb
                recording = self.recording

            t.send_rc_control(0, fb, int(out_z), 0)

            if recording:
                vgy = safe_speed(t, "y")
                with self.lock:
                    self.samples.append(vgy)

            if time.time() - self.last_heartbeat >= HEARTBEAT_S:
                self.last_heartbeat = time.time()

            self.status.update(height=height, err_z=err_z, out_z=out_z, battery=battery)

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, DT - elapsed))


class OperatorInputThread(threading.Thread):
    """Polls keyboard the whole session: E=emergency (any time), L=land
    request (sets land_event), G=go (sets go_event). get_active_loop() is
    a callable returning the current ControlLoopThread or None."""

    def __init__(self, get_active_loop, land_event, go_event):
        super().__init__(daemon=True)
        self.get_active_loop = get_active_loop
        self.land_event = land_event
        self.go_event = go_event
        self.running = True

    def run(self):
        import keyboard
        while self.running:
            if keyboard.is_pressed('e'):
                loop = self.get_active_loop()
                if loop is not None:
                    loop.emergency_flag = True
                time.sleep(0.5)
            if keyboard.is_pressed('l'):
                self.land_event.set()
                time.sleep(0.3)
            if keyboard.is_pressed('g'):
                self.go_event.set()
                time.sleep(0.3)
            time.sleep(0.05)


def wait_for_go_or_land(land_event, go_event):
    print("  Waiting for G (go) or L (land)...")
    while True:
        if land_event.is_set():
            land_event.clear()
            return 'land'
        if go_event.is_set():
            go_event.clear()
            return 'go'
        time.sleep(0.05)


def do_forward_segment(loop, rc, duration, land_event):
    """Returns (samples, interrupted_bool)."""
    loop.set_fb(rc)
    loop.start_recording()
    start = time.time()
    interrupted = False
    while time.time() - start < duration:
        if land_event.is_set():
            interrupted = True
            break
        if loop.land_flag:  # autonomous abort (battery/z-saturation)
            interrupted = True
            break
        time.sleep(0.05)
    samples = loop.stop_recording()
    loop.set_fb(0)
    return samples, interrupted


def takeoff_and_start_loop(tello):
    tello.takeoff()
    time.sleep(POST_TAKEOFF_SETTLE_S)
    loop = ControlLoopThread(tello)
    loop.start()
    return loop


def land_and_stop_loop(loop):
    loop.set_fb(0)
    time.sleep(0.3)
    loop.land_flag = True
    time.sleep(3)
    loop.running = False


def main():
    tello = Tello()
    tello.connect()
    battery = tello.get_battery()
    print(f"Battery: {battery}%")
    if battery < 30:
        print("Battery too low to safely run this test. Aborting.")
        return

    active_loop_holder = {"loop": None}
    land_event = threading.Event()
    go_event = threading.Event()
    operator = OperatorInputThread(lambda: active_loop_holder["loop"], land_event, go_event)
    operator.start()

    trials = [(rc, n) for rc in range(RC_START, RC_END + 1, RC_STEP)
              for n in range(1, TRIALS_PER_RC + 1)]

    results = {}  # rc_value -> list of trial avg velocities
    os.makedirs(os.path.dirname(OUTPUT_CSV), exist_ok=True)
    write_header = not os.path.exists(OUTPUT_CSV)

    loop = takeoff_and_start_loop(tello)
    active_loop_holder["loop"] = loop

    i = 0
    while i < len(trials):
        rc, trial_num = trials[i]
        print(f"\nRC={rc} trial {trial_num}/{TRIALS_PER_RC} (overall {i + 1}/{len(trials)})")

        samples, interrupted = do_forward_segment(loop, rc, MOVE_S, land_event)

        if interrupted:
            if loop.abort_reason in ("battery", "z_saturation"):
                print(f"Autonomous abort ({loop.abort_reason}) mid-trial. "
                      f"Trial discarded. Stopping test entirely — not safe to resume.")
                land_and_stop_loop(loop)
                active_loop_holder["loop"] = None
                break
            else:
                print("Landed mid-trial by request. Trial discarded, will retry after resume.")
                land_and_stop_loop(loop)
                active_loop_holder["loop"] = None
                land_event.clear()
                input("Reposition the drone to a safe start position, then press G to resume...")
                go_event.clear()
                while not go_event.is_set():
                    time.sleep(0.05)
                go_event.clear()
                loop = takeoff_and_start_loop(tello)
                active_loop_holder["loop"] = loop
                continue  # retry same i, same trial

        avg_v = statistics.mean(abs(v) for v in samples) if samples else 0.0
        results.setdefault(rc, []).append(avg_v)
        print(f"  avg |vgy| = {avg_v:.2f} cm/s ({len(samples)} samples) "
              f"battery={loop.status['battery']}%")

        with open(OUTPUT_CSV, "a", newline="") as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow(["rc_value", "trial_number", "avg_velocity_cm_s"])
                write_header = False
            writer.writerow([rc, trial_num, f"{avg_v:.3f}"])

        if trial_num == TRIALS_PER_RC:
            block_avg = statistics.mean(results[rc])
            print(f"=== RC={rc} complete: avg over {len(results[rc])} trials "
                  f"= {block_avg:.2f} cm/s ===")

        i += 1
        if i >= len(trials):
            break  # done, land below, no need to wait

        action = wait_for_go_or_land(land_event, go_event)
        if action == 'land':
            print("Landing by request.")
            land_and_stop_loop(loop)
            active_loop_holder["loop"] = None
            input("Reposition the drone to a safe start position, then press G to resume...")
            go_event.clear()
            while not go_event.is_set():
                time.sleep(0.05)
            go_event.clear()
            loop = takeoff_and_start_loop(tello)
            active_loop_holder["loop"] = loop
        # if 'go': stays airborne, loop continues to next i

    if active_loop_holder["loop"] is not None:
        land_and_stop_loop(active_loop_holder["loop"])
        active_loop_holder["loop"] = None

    operator.running = False

    print("\n=== FINAL RESULTS ===")
    for rc_value in sorted(results):
        avg_v = statistics.mean(results[rc_value])
        print(f"rc={rc_value:>3}  ->  avg |velocity| = {avg_v:.2f} cm/s "
              f"({len(results[rc_value])} trials)")
    print(f"\nWritten to {OUTPUT_CSV}")


if __name__ == "__main__":
    main()