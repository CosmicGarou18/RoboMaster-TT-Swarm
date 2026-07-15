"""
manual_pid_tune_live.py

AUTOMATIC mid-flight PID tuner. Hover trial only. Same control loop,
same safety mechanisms, same dashboard/plot as the manual version -- the
only thing replaced is WHO changes the gains: instead of Q/W/A/S/Z/X
keypresses, an AutoTuner thread automatically walks a shuffled grid of
(kp, ki, kd) combinations, holding each one for AUTO_DWELL_S seconds,
scoring it by mean absolute error over that dwell, logging the result, and
moving to the next combo. L (land) and E (emergency) remain as a manual
kill switch on their own thread (SafetyKeys) -- automating the tuning does
not remove the human override.

ARCHITECTURE NOTE (read this before touching threading):
matplotlib's GUI event loop must own the main thread on most backends,
especially Windows. So the drone control loop (20Hz, timing-sensitive --
heartbeat, rc commands) runs in its OWN background thread
(ControlLoopThread), and main() just builds the plot and blocks on
plt.show(). Do NOT move plt.pause()/plt.show() calls inside the control
loop itself -- that reintroduces exactly the kind of timing jitter that
caused the heartbeat/takeoff regression documented in the swarm project.

- Two selectable tests, chosen at startup via terminal input:
    z        -> vertical height, TOF sensor, single error/output
    lateral  -> horizontal shift, using the drone's own reported velocity
                (vgx/vgy via get_speed_x()/get_speed_y()) integrated ONCE
                into position, tuned TOGETHER as one test with ONE shared
                set of gains. Drift is linear (single integration), not
                quadratic (no more accel double-integration).
- Height   : TOF sensor only (get_distance_tof), never get_height().
- Gain changes: every automatic combo switch FREEZES output for
  EDIT_FREEZE_MS before resuming with the new gains, and resets both PID
  integrals -- same protection the manual version gave a keypress.
- Closing the plot window = safe land request, same as pressing L.
- Heartbeat is NOT suspended during freeze, and runs on its own thread now
  so it can never be blocked by plotting.

Deps: pip install djitellopy keyboard matplotlib
"""

import time
import threading
import csv
import os
import random
import itertools
from collections import deque
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

from djitellopy import Tello

# ---------------------------------------------------------------------------
# CONFIG
# ---------------------------------------------------------------------------
AXIS = None
AXIS_OPTIONS = {"1": "z", "2": "lateral"}

TARGET_HEIGHT_CM = 100.0
CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ

HEARTBEAT_INTERVAL_S = 5.0
SATURATION_LIMIT = 100
SATURATION_ABORT_S = 1.5

EDIT_FREEZE_MS = 400
BATTERY_ABORT_PCT = 15
VELOCITY_DRIFT_WARN_S = 15.0  # single integration drifts slower than double --
                               # longer grace period than the old IMU_DRIFT_WARN_S=8,
                               # but still not indefinite. Verify this on real
                               # flights before trusting the number.

GAINS_BY_AXIS = {
    "z": {"kp": 0.30, "ki": 0.00, "kd": 0.10},
    "lateral": {"kp": 7.0, "ki": 0.66666666, "kd": 1.222222222},
}
GAINS = None
HOLD_GAINS = {"kp": 0.20, "ki": 0.00, "kd": 0.05}

# Auto-tune grid -- same shuffled-grid-search philosophy as pid_tuning.py's
# PID sweep, just smaller since this is real hardware, not simulation.
# NOT validated against real flight yet -- these are starting ranges, same
# caveat as every other seed-value table in this project.
AUTO_KP_VALUES = np.linspace(3, 7, 5)
AUTO_KI_VALUES = np.linspace(0, 1, 4)
AUTO_KD_VALUES = np.linspace(1, 3, 10)
AUTO_DWELL_S = 4.0          # seconds held per combo before scoring + moving on
AUTO_SHUFFLE_GRID = True    # shuffle combo order, matches pid_tuning.py convention

gains_lock = threading.Lock()

OUTPUT_DIR = "output"
os.makedirs(OUTPUT_DIR, exist_ok=True)

PLOT_WINDOW_S = 12                       # seconds of history shown on screen
PLOT_MAXLEN = int(CONTROL_HZ * PLOT_WINDOW_S)
PLOT_REFRESH_MS = 150                    # ~6-7Hz redraw, deliberately slower
                                          # than the 20Hz control loop so the
                                          # GUI never becomes the bottleneck


def choose_axis() -> str:
    print("Select test to run:")
    print("  1) z        -- vertical height PID (TOF, safe, no drift)")
    print("  2) lateral  -- horizontal shift PID, x+y together (vgx/vgy, single-integrated)")
    while True:
        choice = input("Enter 1 or 2: ").strip()
        if choice in AXIS_OPTIONS:
            return AXIS_OPTIONS[choice]
        print(f"Invalid choice: {choice!r}. Try again.")


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


def safe_int_battery(tello: Tello, last_known):
    """Never trust a raw response as data without checking its shape --
    same principle as HeightReader. A stale/out-of-sync response string
    (e.g. 'ok' landing where a battery number was expected) must not crash
    the loop; hold last known value instead."""
    try:
        return int(tello.query_battery())
    except (ValueError, TypeError):
        return last_known


def safe_speed(getter, last_known):
    """Same defensive pattern for vgx/vgy reads. These come from the parsed
    UDP state stream, not a command/response pair, so the 'stale response'
    failure mode is less likely -- but a dropped/malformed state packet or
    a momentary None is still possible, and this is a control-loop input
    now, not just a monitor. Hold last known value on any failure."""
    try:
        return float(getter())
    except (ValueError, TypeError):
        return last_known


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
        print(f"  Last action : {state_ref['last_action']}")
    print("=" * 40)
    print("AUTO TUNING -- gains change automatically. L land  E EMERGENCY (kill switch only)")
    print("Close the plot window to land safely.")


class HeightReader:
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


class VelocityAxisState:
    """Single-integrates a real velocity reading (vgx or vgy, already
    sensor-fused by the drone) into a position estimate. Replaces the old
    accel double-integration (accel -> vel -> pos): here the velocity
    itself is measured, not derived, so only one integration step happens.
    Still drifts -- any small bias in vgx/vgy accumulates over time -- but
    linearly, not quadratically. Still not a substitute for a real position
    sensor (mission pad), which remains excluded per current instruction."""

    def __init__(self, label: str):
        self.label = label
        self.pos = 0.0
        self.start_time = time.time()
        self.last_velocity = 0.0

    def update(self, velocity_cms: float, dt: float):
        self.last_velocity = velocity_cms
        self.pos += velocity_cms * dt

    def drift_seconds(self) -> float:
        return time.time() - self.start_time


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
# PLOT DATA -- thread-safe ring buffers, written by control thread,
# read by the main-thread animation callback
# ---------------------------------------------------------------------------
plot_lock = threading.Lock()
plot_data = {
    "t": deque(maxlen=PLOT_MAXLEN),
    "error_1": deque(maxlen=PLOT_MAXLEN),
    "error_2": deque(maxlen=PLOT_MAXLEN),
    "output_1": deque(maxlen=PLOT_MAXLEN),
    "output_2": deque(maxlen=PLOT_MAXLEN),
}
plot_start_time = None


def push_plot_sample(error_1, error_2, output_1, output_2):
    global plot_start_time
    now = time.time()
    if plot_start_time is None:
        plot_start_time = now
    with plot_lock:
        plot_data["t"].append(now - plot_start_time)
        plot_data["error_1"].append(error_1 if error_1 is not None else float("nan"))
        plot_data["error_2"].append(error_2 if error_2 is not None else float("nan"))
        plot_data["output_1"].append(output_1 if output_1 is not None else float("nan"))
        plot_data["output_2"].append(output_2 if output_2 is not None else float("nan"))


def snapshot_plot_data():
    with plot_lock:
        return {k: list(v) for k, v in plot_data.items()}


# ---------------------------------------------------------------------------
# AUTO TUNER THREAD -- automatically walks a grid of (kp, ki, kd) combos,
# dwelling on each one, scoring it, logging it, then moving on. Replaces
# manual keypress-driven gain nudging. The freeze mechanism (trigger_freeze)
# and integral reset are reused exactly as they were for the manual keys --
# every automatic gain change gets the same freeze-then-resume protection a
# keypress got, so a jump from one combo to the next never hits a stale
# error/integral instantly.
# ---------------------------------------------------------------------------
class AutoTuner(threading.Thread):
    def __init__(self, gains_dict, pids, csv_writer, csv_file, state_ref, combos, dwell_s):
        super().__init__(daemon=True)
        self.gains_dict = gains_dict
        self.pids = pids
        self.csv_writer = csv_writer
        self.csv_file = csv_file
        self.state_ref = state_ref
        self.combos = combos
        self.dwell_s = dwell_s

    def _aborted(self) -> bool:
        return self.state_ref["stop"] or self.state_ref["land"] or self.state_ref["emergency"]

    def run(self):
        for i, (kp, ki, kd) in enumerate(self.combos):
            if self._aborted():
                break

            with gains_lock:
                self.gains_dict["kp"] = kp
                self.gains_dict["ki"] = ki
                self.gains_dict["kd"] = kd

            for pid in self.pids:
                pid.reset_integral()
            trigger_freeze()

            self.state_ref["last_action"] = (
                f"AUTO [{i+1}/{len(self.combos)}] kp={kp} ki={ki} kd={kd}"
            )

            error_samples_1 = []
            error_samples_2 = []
            dwell_start = time.time()
            while time.time() - dwell_start < self.dwell_s:
                if self._aborted():
                    break
                e1 = self.state_ref.get("error_1")
                e2 = self.state_ref.get("error_2")
                if e1 is not None:
                    error_samples_1.append(abs(e1))
                if e2 is not None:
                    error_samples_2.append(abs(e2))
                time.sleep(0.05)

            cost_1 = round(sum(error_samples_1) / len(error_samples_1), 3) if error_samples_1 else None
            cost_2 = round(sum(error_samples_2) / len(error_samples_2), 3) if error_samples_2 else None
            self._log_result(kp, ki, kd, cost_1, cost_2)

            if self._aborted():
                break

        if not (self.state_ref["stop"] or self.state_ref["emergency"]):
            self.state_ref["last_action"] = "AUTO TUNE COMPLETE -- landing"
            self.state_ref["land"] = True

    def _log_result(self, kp, ki, kd, cost_1, cost_2):
        row = [ts(), AXIS, kp, ki, kd, cost_1, cost_2,
               self.state_ref.get("last_height"), self.state_ref.get("last_battery")]
        self.csv_writer.writerow(row)
        self.csv_file.flush()  # flush every row -- if this crashes mid-sequence,
                                # partial results are still on disk, not lost


# ---------------------------------------------------------------------------
# SAFETY KEYS THREAD -- L (land) and E (emergency) ONLY. This is a manual
# override kept intentionally even though gain tuning is now automatic --
# an automated loop still needs a human-operable kill switch. Does NOT
# touch gains at all.
# ---------------------------------------------------------------------------
class SafetyKeys(threading.Thread):
    def __init__(self, state_ref):
        super().__init__(daemon=True)
        self.state_ref = state_ref
        self._last_press = {}

    def _debounced(self, key, cooldown=1.0) -> bool:
        now = time.time()
        last = self._last_press.get(key, 0)
        if now - last < cooldown:
            return False
        self._last_press[key] = now
        return True

    def run(self):
        import keyboard

        while not self.state_ref["stop"]:
            try:
                if keyboard.is_pressed("l") and self._debounced("l"):
                    self.state_ref["last_action"] = "L (land requested)"
                    self.state_ref["land"] = True
                elif keyboard.is_pressed("e") and self._debounced("e"):
                    self.state_ref["last_action"] = "E (EMERGENCY requested)"
                    self.state_ref["emergency"] = True
            except Exception as e:
                self.state_ref["last_action"] = f"SafetyKeys error: {e}"
            time.sleep(0.02)


# ---------------------------------------------------------------------------
# CONTROL LOOP -- runs on its own thread so plotting can own the main thread
# ---------------------------------------------------------------------------
class ControlLoopThread(threading.Thread):
    def __init__(self, tello, state_ref, csv_file):
        super().__init__(daemon=True)
        self.tello = tello
        self.state_ref = state_ref
        self.csv_file = csv_file

        self.height_reader = HeightReader()
        if AXIS == "z":
            self.pid_1 = PID(GAINS)
            self.pid_2 = None
        else:
            self.pid_1 = PID(GAINS)  # left/right
            self.pid_2 = PID(GAINS)  # forward/back
        self.hold_pid = PID(HOLD_GAINS)
        self.vel_lr = VelocityAxisState("lr")
        self.vel_fb = VelocityAxisState("fb")

    def pids_for_reset(self):
        return [self.pid_1] if self.pid_2 is None else [self.pid_1, self.pid_2]

    def run(self):
        tello = self.tello
        state_ref = self.state_ref

        send(tello, "takeoff")
        time.sleep(3)

        last_heartbeat = time.time()
        saturation_start = None

        try:
            while not state_ref["stop"]:
                loop_start = time.time()

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

                height_cm = self.height_reader.read(tello)
                state_ref["last_height"] = height_cm
                state_ref["last_battery"] = safe_int_battery(tello, state_ref["last_battery"])

                frozen = in_freeze()
                state_ref["frozen"] = frozen

                if AXIS == "z":
                    error_1 = TARGET_HEIGHT_CM - height_cm
                    state_ref["error_1"] = round(error_1, 2)

                    if frozen:
                        send_rc(tello, 0, 0, 0, 0)
                        out_1 = 0.0
                    else:
                        out_1 = self.pid_1.compute(error_1, DT)
                        send_rc(tello, 0, 0, int(out_1), 0)

                    state_ref["output_1"] = round(out_1, 2)
                    worst_out = abs(out_1)
                    push_plot_sample(error_1, None, out_1, None)

                else:
                    vgx = safe_speed(tello.get_speed_x, self.vel_lr.last_velocity)
                    vgy = safe_speed(tello.get_speed_y, self.vel_fb.last_velocity)
                    self.vel_lr.update(vgx, DT)
                    self.vel_fb.update(vgy, DT)

                    error_lr = -self.vel_lr.pos
                    error_fb = -self.vel_fb.pos
                    state_ref["error_1"] = round(error_lr, 2)
                    state_ref["error_2"] = round(error_fb, 2)

                    if self.vel_lr.drift_seconds() > VELOCITY_DRIFT_WARN_S:
                        state_ref["last_action"] = (
                            f"WARNING: position drift > {VELOCITY_DRIFT_WARN_S:.0f}s -- "
                            f"integrated estimate may no longer be trustworthy"
                        )

                    if frozen:
                        send_rc(tello, 0, 0, 0, 0)
                        out_lr = 0.0
                        out_fb = 0.0
                    else:
                        out_lr = self.pid_1.compute(error_lr, DT)
                        out_fb = self.pid_2.compute(error_fb, DT)
                        hold_out = self.hold_pid.compute(TARGET_HEIGHT_CM - height_cm, DT)
                        send_rc(tello, int(out_lr), int(out_fb), int(hold_out), 0)

                    state_ref["output_1"] = round(out_lr, 2)
                    state_ref["output_2"] = round(out_fb, 2)
                    worst_out = max(abs(out_lr), abs(out_fb))
                    push_plot_sample(error_lr, error_fb, out_lr, out_fb)

                if worst_out >= SATURATION_LIMIT:
                    state_ref["saturated"] = True
                    if saturation_start is None:
                        saturation_start = time.time()
                    elif time.time() - saturation_start >= SATURATION_ABORT_S:
                        state_ref["last_action"] = (
                            f"SATURATION ABORT: pinned {SATURATION_ABORT_S}s+ -- landing"
                        )
                        send_rc(tello, 0, 0, 0, 0)
                        time.sleep(0.3)
                        send(tello, "land")
                        break
                else:
                    state_ref["saturated"] = False
                    saturation_start = None

                if state_ref["last_battery"] is not None and state_ref["last_battery"] < BATTERY_ABORT_PCT:
                    state_ref["last_action"] = f"BATTERY ABORT: {state_ref['last_battery']}%"
                    send_rc(tello, 0, 0, 0, 0)
                    time.sleep(0.3)
                    send(tello, "land")
                    break

                elapsed = time.time() - loop_start
                time.sleep(max(0.0, DT - elapsed))

        finally:
            state_ref["stop"] = True
            self.csv_file.close()


# ---------------------------------------------------------------------------
# DASHBOARD REFRESH THREAD -- separate from control loop so a slow terminal
# repaint can never delay an rc command
# ---------------------------------------------------------------------------
class DashboardThread(threading.Thread):
    def __init__(self, state_ref):
        super().__init__(daemon=True)
        self.state_ref = state_ref

    def run(self):
        while not self.state_ref["stop"]:
            render_dashboard(GAINS, self.state_ref)
            time.sleep(0.1)


# ---------------------------------------------------------------------------
# MAIN -- sets up the drone + threads, then hands the main thread to
# matplotlib. plt.show() blocks here; closing the window requests a land.
# ---------------------------------------------------------------------------
def main():
    global AXIS, GAINS

    AXIS = choose_axis()
    GAINS = GAINS_BY_AXIS[AXIS]
    csv_path = os.path.join(OUTPUT_DIR, f"auto_tune_{AXIS}.csv")

    tello = Tello()
    send(tello, "command")

    battery = safe_int_battery(tello, None)
    if battery is None or battery < BATTERY_ABORT_PCT:
        clear_screen()
        print(f"Battery too low or unreadable ({battery}). Aborting before takeoff.")
        return

    state_ref = {"land": False, "emergency": False, "stop": False,
                 "error_1": None, "error_2": None,
                 "output_1": None, "output_2": None,
                 "last_height": None, "last_battery": battery,
                 "frozen": False, "saturated": False, "last_action": None}

    csv_file = open(csv_path, "a", newline="")
    csv_writer = csv.writer(csv_file)
    if os.stat(csv_path).st_size == 0:
        csv_writer.writerow(["timestamp", "axis", "kp", "ki", "kd",
                              "cost_1_mean_abs_error", "cost_2_mean_abs_error",
                              "height_cm", "battery"])

    combos = list(itertools.product(AUTO_KP_VALUES, AUTO_KI_VALUES, AUTO_KD_VALUES))
    if AUTO_SHUFFLE_GRID:
        random.shuffle(combos)

    control_thread = ControlLoopThread(tello, state_ref, csv_file)
    tuner = AutoTuner(GAINS, control_thread.pids_for_reset(), csv_writer, csv_file,
                       state_ref, combos, AUTO_DWELL_S)
    safety_keys = SafetyKeys(state_ref)
    dashboard = DashboardThread(state_ref)

    safety_keys.start()
    tuner.start()
    dashboard.start()
    control_thread.start()

    # --- matplotlib setup, MAIN THREAD ONLY ---
    fig, (ax_err, ax_out) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    fig.suptitle(f"Live PID tuning -- {AXIS}")

    line_err1, = ax_err.plot([], [], label="error_1" if AXIS == "lateral" else "error (height)")
    line_err2, = ax_err.plot([], [], label="error_2 (fb)") if AXIS == "lateral" else (None,)
    ax_err.set_ylabel("Error")
    ax_err.legend(loc="upper right")
    ax_err.grid(True, alpha=0.3)

    line_out1, = ax_out.plot([], [], label="output_1" if AXIS == "lateral" else "output")
    line_out2, = ax_out.plot([], [], label="output_2 (fb)") if AXIS == "lateral" else (None,)
    ax_out.set_ylabel("Output (rc value)")
    ax_out.set_xlabel("Time (s)")
    ax_out.axhline(SATURATION_LIMIT, color="red", linestyle="--", alpha=0.4)
    ax_out.axhline(-SATURATION_LIMIT, color="red", linestyle="--", alpha=0.4)
    ax_out.legend(loc="upper right")
    ax_out.grid(True, alpha=0.3)

    def on_close(event):
        # Closing the plot window is treated the same as pressing L.
        state_ref["last_action"] = "Plot window closed (land requested)"
        state_ref["land"] = True

    fig.canvas.mpl_connect("close_event", on_close)

    def update(_frame):
        data = snapshot_plot_data()
        t = data["t"]
        if not t:
            return line_err1, line_out1
        line_err1.set_data(t, data["error_1"])
        line_out1.set_data(t, data["output_1"])
        if AXIS == "lateral":
            line_err2.set_data(t, data["error_2"])
            line_out2.set_data(t, data["output_2"])
        for ax in (ax_err, ax_out):
            ax.set_xlim(max(0, t[-1] - PLOT_WINDOW_S), max(PLOT_WINDOW_S, t[-1]))
        ax_err.relim(); ax_err.autoscale_view(scalex=False)
        ax_out.relim(); ax_out.autoscale_view(scalex=False)
        artists = [line_err1, line_out1]
        if AXIS == "lateral":
            artists += [line_err2, line_out2]
        return artists

    ani = animation.FuncAnimation(fig, update, interval=PLOT_REFRESH_MS,
                                   cache_frame_data=False)

    try:
        plt.show()  # BLOCKS here until window closed -- this is expected
    except KeyboardInterrupt:
        state_ref["last_action"] = "Ctrl+C (land requested)"
        state_ref["land"] = True

    # window closed or Ctrl+C -- give the control thread a moment to land
    control_thread.join(timeout=10)
    state_ref["stop"] = True
    clear_screen()
    print("Session ended.")


if __name__ == "__main__":
    main()