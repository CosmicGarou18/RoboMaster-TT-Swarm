"""
manual_pid_tune_live.py

Live mid-flight PID tuner. Hover trial only.

- Two selectable tests, chosen at startup via terminal input:
    z        -> vertical height, TOF sensor, single error/output
    lateral  -> horizontal shift, IMU accel_x (left/right) AND accel_y
                (forward/back) tuned TOGETHER as one test with ONE shared
                set of gains. Two separate PID controllers run (one per
                axis) but both read the SAME gains dict, so a Q/W/A/S/Z/X
                keypress changes both simultaneously. Height is held fixed
                via TOF during the lateral test (not swept).
- Height axis   : TOF sensor (get_distance_tof), NOT get_height(). Same
                   HeightReader outlier/sentinel filter as prior scripts.
- Lateral       : IMU acceleration-domain error (ORIGINAL method, no mission
                   pad). Dead reckoning drifts past ~8s — known, accepted.
                   This applies to BOTH x and y independently; each axis
                   has its own drift clock.
- Gain changes  : any Q/W/A/S/Z/X keypress FREEZES control output
                   (rc 0 0 0 0 hold) for EDIT_FREEZE_MS before resuming with
                   the new gains. Prevents a Kp jump hitting a stale
                   error/integral instantly.
- Dashboard     : clears screen and reprints PID gains + battery + height +
                   error/output every tick. No scrolling console output.
- Heartbeat is NOT suspended during the freeze window.

Deps: pip install djitellopy keyboard
"""

import time
import threading
import csv
import os
from datetime import datetime

from djitellopy import Tello

# ---------------------------------------------------------------------------
# CONFIG
# ---------------------------------------------------------------------------
AXIS = None  # set at runtime via choose_axis() -- "z" or "lateral"
AXIS_OPTIONS = {
    "1": "z",         # vertical height, TOF, safe, no drift
    "2": "lateral",   # horizontal shift (x+y together), IMU accel, drift-prone
}

TARGET_HEIGHT_CM = 100.0
CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ

HEARTBEAT_INTERVAL_S = 5.0     # battery? while idle-ish; must survive freezes
SATURATION_LIMIT = 100
SATURATION_ABORT_S = 1.5

EDIT_FREEZE_MS = 400            # freeze window after any gain keypress
BATTERY_ABORT_PCT = 15
IMU_DRIFT_WARN_S = 8.0          # dead reckoning considered unreliable past this

# starting gains -- same seed philosophy as prior scripts, not final values.
GAINS_BY_AXIS = {
    "z": {"kp": 0.30, "ki": 0.00, "kd": 0.10},
    "lateral": {"kp": 0.20, "ki": 0.00, "kd": 0.05},  # conservative: drift-prone axis
}
GAINS = None  # set to GAINS_BY_AXIS[AXIS] at runtime
HOLD_GAINS = {"kp": 0.20, "ki": 0.00, "kd": 0.05}  # height-hold gains, used during lateral test

gains_lock = threading.Lock()

OUTPUT_DIR = "output"
os.makedirs(OUTPUT_DIR, exist_ok=True)


def choose_axis() -> str:
    """Terminal menu -- blocks until a valid choice is made."""
    print("Select test to run:")
    print("  1) z        -- vertical height PID (TOF, safe, no drift)")
    print("  2) lateral  -- horizontal shift PID, x+y together (IMU, drift-prone)")
    while True:
        choice = input("Enter 1 or 2: ").strip()
        if choice in AXIS_OPTIONS:
            return AXIS_OPTIONS[choice]
        print(f"Invalid choice: {choice!r}. Try again.")


# ---------------------------------------------------------------------------
# COMMAND WRAPPER -- no console output, dashboard owns the screen
# ---------------------------------------------------------------------------
def ts():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def send(tello: Tello, cmd: str):
    try:
        return tello.send_command_with_return(cmd, timeout=7)
    except Exception:
        return None


def send_rc(tello: Tello, lr, fb, ud, yaw):
    try:
        tello.send_rc_control(lr, fb, ud, yaw)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# DASHBOARD -- clear screen, reprint every tick, no scrolling
# ---------------------------------------------------------------------------
def clear_screen():
    os.system("cls" if os.name == "nt" else "clear")


def render_dashboard(gains_dict, state_ref):
    with gains_lock:
        kp, ki, kd = gains_dict["kp"], gains_dict["ki"], gains_dict["kd"]
    clear_screen()
    print("=" * 40)
    print(f"  TEST     : {AXIS}")
    print(f"  Kp       : {kp}")
    print(f"  Ki       : {ki}")
    print(f"  Kd       : {kd}")
    print(f"  Battery  : {state_ref.get('last_battery')}%")
    print(f"  Height   : {state_ref.get('last_height')} cm (TOF)")
    if AXIS == "z":
        print(f"  Error    : {state_ref.get('error_1')}")
        print(f"  Output   : {state_ref.get('output_1')}")
    else:
        print(f"  Error  lr (left/right)  : {state_ref.get('error_1')}")
        print(f"  Error  fb (fwd/back)    : {state_ref.get('error_2')}")
        print(f"  Output lr               : {state_ref.get('output_1')}")
        print(f"  Output fb               : {state_ref.get('output_2')}")
    if state_ref.get("frozen"):
        print("  [FROZEN -- output held after gain edit]")
    if state_ref.get("saturated"):
        print("  [WARNING: output saturated]")
    if state_ref.get("last_action"):
        print(f"  Last key : {state_ref['last_action']}")
    print("=" * 40)
    print("Q/W Kp-/+  A/S Ki-/+  Z/X Kd-/+  R reset-I  P log  L land  E EMERGENCY")


# ---------------------------------------------------------------------------
# HEIGHT -- TOF ONLY (never get_height())
# ---------------------------------------------------------------------------
class HeightReader:
    """Rejects 8888 sentinel, rejects out-of-range, rejects >50cm single-tick
    jumps vs last good, holds last good on any reject."""

    def __init__(self):
        self.last_good = None

    def read(self, tello: Tello) -> float:
        try:
            raw = tello.get_distance_tof()
        except Exception:
            return self.last_good if self.last_good is not None else 0.0

        if raw == 8888:
            return self.last_good if self.last_good is not None else 0.0
        if not (1 < raw < 800):
            return self.last_good if self.last_good is not None else 0.0
        if self.last_good is not None and abs(raw - self.last_good) > 50:
            return self.last_good
        self.last_good = raw
        return raw


# ---------------------------------------------------------------------------
# LATERAL -- IMU ACCELERATION-DOMAIN ERROR, one instance per axis (x, y)
# ---------------------------------------------------------------------------
class ImuAxisState:
    """Double-integrates accel into a drift-prone velocity/position estimate.
    Unreliable past ~IMU_DRIFT_WARN_S seconds. One instance = one direction
    (x or y) -- the lateral test runs two of these independently."""

    def __init__(self, label: str):
        self.label = label
        self.vel = 0.0
        self.pos = 0.0
        self.start_time = time.time()

    def update(self, accel_cms2: float, dt: float):
        self.vel += accel_cms2 * dt
        self.pos += self.vel * dt

    def drift_seconds(self) -> float:
        return time.time() - self.start_time


# ---------------------------------------------------------------------------
# PID -- reads gains fresh every tick from the shared dict
# ---------------------------------------------------------------------------
class PID:
    def __init__(self, gains_dict):
        self.gains_dict = gains_dict
        self.integral = 0.0
        self.prev_error = 0.0

    def reset_integral(self):
        self.integral = 0.0

    def compute(self, error: float, dt: float) -> float:
        with gains_lock:
            kp = self.gains_dict["kp"]
            ki = self.gains_dict["ki"]
            kd = self.gains_dict["kd"]
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        out = kp * error + ki * self.integral + kd * derivative
        return max(-SATURATION_LIMIT, min(SATURATION_LIMIT, out))


# ---------------------------------------------------------------------------
# FREEZE STATE -- shared between keyboard thread and control loop
# ---------------------------------------------------------------------------
freeze_until = 0.0
freeze_lock = threading.Lock()


def trigger_freeze():
    global freeze_until
    with freeze_lock:
        freeze_until = time.time() + (EDIT_FREEZE_MS / 1000.0)


def in_freeze() -> bool:
    with freeze_lock:
        return time.time() < freeze_until


# ---------------------------------------------------------------------------
# KEYBOARD TUNER THREAD
# ---------------------------------------------------------------------------
class KeyboardTuner(threading.Thread):
    def __init__(self, gains_dict, pids, csv_writer, state_ref):
        """pids: list of PID instances to reset together on R (one for z,
        two -- lr and fb -- for lateral)."""
        super().__init__(daemon=True)
        self.gains_dict = gains_dict
        self.pids = pids
        self.csv_writer = csv_writer
        self.state_ref = state_ref
        self._last_press = {}

    def _debounced(self, key, cooldown=0.15) -> bool:
        now = time.time()
        last = self._last_press.get(key, 0)
        if now - last < cooldown:
            return False
        self._last_press[key] = now
        return True

    def run(self):
        import keyboard  # imported here so the rest of the file works even
                          # without the lib installed, until this thread starts

        while not self.state_ref["stop"]:
            try:
                if keyboard.is_pressed("q") and self._debounced("q"):
                    self._nudge("kp", -0.05)
                elif keyboard.is_pressed("w") and self._debounced("w"):
                    self._nudge("kp", +0.05)
                elif keyboard.is_pressed("a") and self._debounced("a"):
                    self._nudge("ki", -0.01)
                elif keyboard.is_pressed("s") and self._debounced("s"):
                    self._nudge("ki", +0.01)
                elif keyboard.is_pressed("z") and self._debounced("z"):
                    self._nudge("kd", -0.05)
                elif keyboard.is_pressed("x") and self._debounced("x"):
                    self._nudge("kd", +0.05)
                elif keyboard.is_pressed("r") and self._debounced("r"):
                    for pid in self.pids:
                        pid.reset_integral()
                    self.state_ref["last_action"] = "R (integral reset)"
                elif keyboard.is_pressed("p") and self._debounced("p"):
                    self._log_snapshot()
                elif keyboard.is_pressed("l") and self._debounced("l", 1.0):
                    self.state_ref["last_action"] = "L (land requested)"
                    self.state_ref["land"] = True
                elif keyboard.is_pressed("e") and self._debounced("e", 1.0):
                    self.state_ref["last_action"] = "E (EMERGENCY requested)"
                    self.state_ref["emergency"] = True
            except Exception as e:
                self.state_ref["last_action"] = f"KeyboardTuner error: {e}"
            time.sleep(0.02)

    def _nudge(self, key, delta):
        with gains_lock:
            self.gains_dict[key] = round(self.gains_dict[key] + delta, 4)
            snapshot = dict(self.gains_dict)
        self.state_ref["last_action"] = f"{key} -> {snapshot[key]}"
        trigger_freeze()

    def _log_snapshot(self):
        with gains_lock:
            snapshot = dict(self.gains_dict)
        row = [ts(), AXIS, snapshot["kp"], snapshot["ki"], snapshot["kd"],
               self.state_ref.get("error_1"), self.state_ref.get("error_2"),
               self.state_ref.get("last_height"), self.state_ref.get("last_battery")]
        self.csv_writer.writerow(row)
        self.state_ref["last_action"] = "P (snapshot logged)"


# ---------------------------------------------------------------------------
# MAIN CONTROL LOOP
# ---------------------------------------------------------------------------
def main():
    global AXIS, GAINS

    AXIS = choose_axis()
    GAINS = GAINS_BY_AXIS[AXIS]
    csv_path = os.path.join(OUTPUT_DIR, f"manual_tune_live_{AXIS}.csv")

    tello = Tello()
    send(tello, "command")

    battery = tello.get_battery()
    if battery < BATTERY_ABORT_PCT:
        clear_screen()
        print(f"Battery too low to start ({battery}%). Aborting before takeoff.")
        return

    height_reader = HeightReader()

    if AXIS == "z":
        pid_1 = PID(GAINS)
        pid_2 = None
        pids_for_reset = [pid_1]
    else:
        # lateral: two independent controllers, SAME shared gains dict
        pid_1 = PID(GAINS)  # left/right (lr)
        pid_2 = PID(GAINS)  # forward/back (fb)
        pids_for_reset = [pid_1, pid_2]

    hold_pid = PID(HOLD_GAINS)  # height-hold, used only during lateral test

    state_ref = {"land": False, "emergency": False, "stop": False,
                 "error_1": None, "error_2": None,
                 "output_1": None, "output_2": None,
                 "last_height": None, "last_battery": battery,
                 "frozen": False, "saturated": False, "last_action": None}

    csv_file = open(csv_path, "a", newline="")
    csv_writer = csv.writer(csv_file)
    if os.stat(csv_path).st_size == 0:
        csv_writer.writerow(["timestamp", "axis", "kp", "ki", "kd",
                              "error_1", "error_2", "height_cm", "battery"])

    tuner = KeyboardTuner(GAINS, pids_for_reset, csv_writer, state_ref)
    tuner.start()

    send(tello, "takeoff")
    time.sleep(3)

    imu_lr = ImuAxisState("lr")  # left/right, accel_x
    imu_fb = ImuAxisState("fb")  # forward/back, accel_y
    last_heartbeat = time.time()
    saturation_start = None

    try:
        while True:
            loop_start = time.time()

            # --- heartbeat: always runs, freeze or not ---
            if time.time() - last_heartbeat >= HEARTBEAT_INTERVAL_S:
                send(tello, "battery?")
                last_heartbeat = time.time()

            if state_ref["emergency"]:
                send(tello, "emergency")
                break
            if state_ref["land"]:
                send_rc(tello, 0, 0, 0, 0)
                time.sleep(0.3)
                send(tello, "land")
                break

            # --- sensing ---
            height_cm = height_reader.read(tello)
            state_ref["last_height"] = height_cm
            state_ref["last_battery"] = tello.get_battery()

            frozen = in_freeze()
            state_ref["frozen"] = frozen

            if AXIS == "z":
                error_1 = TARGET_HEIGHT_CM - height_cm
                state_ref["error_1"] = round(error_1, 2)

                if frozen:
                    send_rc(tello, 0, 0, 0, 0)
                    out_1 = 0.0
                else:
                    out_1 = pid_1.compute(error_1, DT)
                    send_rc(tello, 0, 0, int(out_1), 0)

                state_ref["output_1"] = round(out_1, 2)
                worst_out = abs(out_1)

            else:  # lateral -- x and y together, one shared gain set
                accel_x = tello.get_acceleration_x() / 100.0
                accel_y = tello.get_acceleration_y() / 100.0
                imu_lr.update(accel_x, DT)
                imu_fb.update(accel_y, DT)

                error_lr = -imu_lr.pos
                error_fb = -imu_fb.pos
                state_ref["error_1"] = round(error_lr, 2)
                state_ref["error_2"] = round(error_fb, 2)

                if imu_lr.drift_seconds() > IMU_DRIFT_WARN_S:
                    state_ref["last_action"] = (
                        f"WARNING: IMU drift > {IMU_DRIFT_WARN_S:.0f}s "
                        f"(t={imu_lr.drift_seconds():.1f}s) -- position estimate "
                        f"no longer trustworthy"
                    )

                if frozen:
                    send_rc(tello, 0, 0, 0, 0)
                    out_lr = 0.0
                    out_fb = 0.0
                else:
                    out_lr = pid_1.compute(error_lr, DT)
                    out_fb = pid_2.compute(error_fb, DT)
                    hold_out = hold_pid.compute(TARGET_HEIGHT_CM - height_cm, DT)
                    send_rc(tello, int(out_lr), int(out_fb), int(hold_out), 0)

                state_ref["output_1"] = round(out_lr, 2)
                state_ref["output_2"] = round(out_fb, 2)
                worst_out = max(abs(out_lr), abs(out_fb))

            # --- saturation watchdog ---
            if worst_out >= SATURATION_LIMIT:
                state_ref["saturated"] = True
                if saturation_start is None:
                    saturation_start = time.time()
                elif time.time() - saturation_start >= SATURATION_ABORT_S:
                    state_ref["last_action"] = (
                        f"SATURATION ABORT: pinned {SATURATION_ABORT_S}s+ -- landing"
                    )
                    render_dashboard(GAINS, state_ref)
                    send_rc(tello, 0, 0, 0, 0)
                    time.sleep(0.3)
                    send(tello, "land")
                    break
            else:
                state_ref["saturated"] = False
                saturation_start = None

            # --- battery watchdog ---
            if state_ref["last_battery"] < BATTERY_ABORT_PCT:
                state_ref["last_action"] = f"BATTERY ABORT: {state_ref['last_battery']}%"
                render_dashboard(GAINS, state_ref)
                send_rc(tello, 0, 0, 0, 0)
                time.sleep(0.3)
                send(tello, "land")
                break

            render_dashboard(GAINS, state_ref)

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, DT - elapsed))

    except KeyboardInterrupt:
        clear_screen()
        print("CTRL+C caught -- landing, not cutting motors.")
        try:
            send_rc(tello, 0, 0, 0, 0)
            time.sleep(0.3)
            send(tello, "land")
        except Exception as e:
            print(f"Landing after Ctrl+C failed: {e} "
                  f"-- if the drone is still airborne, use E for emergency "
                  f"cut or land it manually now.")

    finally:
        state_ref["stop"] = True
        csv_file.close()


if __name__ == "__main__":
    main()