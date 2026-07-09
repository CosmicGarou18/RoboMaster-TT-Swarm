#!/usr/bin/env python3
"""
Tello High-Speed Velocity Calibration & Odometry Profiler
Measures transit times across a fixed 3.3m distance for RC steps 10 through 50.
Features automated post-takeoff pad target centering alignment.
"""

import sys
import time
import csv
import signal
import threading
import numpy as np
from djitellopy import Tello

# Hardcoded Hardware Configurations
PAD_DISTANCE_M = 3.3   # Fixed physical distance between pad centers
PAD_A_ID       = 5     # Launch / Origin pad ID
PAD_B_ID       = 7     # Destination pad ID

# Velocity testing matrix (steps of 10 up to 50)
RC_SPEED_STEPS  = [10, 20, 30, 40, 50]  
TRIALS_PER_STEP = 3        # Number of forward/backward shuttles per speed
LOOP_DT         = 0.01     # 100Hz high-frequency pad polling rate for high-speed tracking

# Global safety variables
danger_flag = threading.Event()
is_airborne = False        # Track flight state to prevent landing exceptions

def ctrl_c_handler(sig, frame):
    print("\n\n[!!!] EMERGENCY INTERCEPT: CTRL+C DETECTED -> Terminating transit & landing...")
    danger_flag.set()

signal.signal(signal.SIGINT, ctrl_c_handler)

def stabilize_over_pad(drone, target_pad_id):
    """
    Commands the drone to actively track and center itself directly over 
    the target pad coordinates, eliminating positional drift.
    """
    if danger_flag.is_set(): return False
    
    print(f"Actively centering precisely over Pad {target_pad_id}...")
    timeout_start = time.time()
    consecutive_centered = 0
    
    # Run loop for up to 6 seconds or until tightly centered over target axis
    while time.time() - timeout_start < 6.0 and not danger_flag.is_set():
        current_pad = drone.get_mission_pad_id()
        if current_pad == target_pad_id:
            # Distances are parsed in centimeters from absolute pad center
            x = drone.get_mission_pad_distance_x()
            y = drone.get_mission_pad_distance_y()
            
            # Proportional adjustments to zero-out horizontal vectors
            fb = -15 if x > 2 else 15 if x < -2 else 0
            lr = -15 if y > 2 else 15 if y < -2 else 0
            
            drone.send_rc_control(lr, fb, 0, 0)
            
            if fb == 0 and lr == 0:
                consecutive_centered += 1
                if consecutive_centered >= 8:  # Confirmed stable over center
                    break
            else:
                consecutive_centered = 0
        else:
            drone.send_rc_control(0, 0, 0, 0)
            consecutive_centered = 0
            
        time.sleep(0.05)
        
    drone.send_rc_control(0, 0, 0, 0)
    time.sleep(0.5)
    return not danger_flag.is_set()

def execute_transit(drone, rc_value, start_pad_id, target_pad_id, direction_label):
    """Executes a single one-way transit between pads, timing the cross duration."""
    if danger_flag.is_set(): return None

    print(f"  Executing {direction_label} run at RC {rc_value}...")
    
    # Send directional pitch command
    drone.send_rc_control(0, rc_value, 0, 0)
    t0 = time.time()
    
    duration = 0.0
    detected = False
    
    # High-frequency poll until target pad enters downward camera view
    while not danger_flag.is_set():
        current_pad = drone.get_mission_pad_id()
        if current_pad == target_pad_id:
            duration = time.time() - t0
            detected = True
            break
            
        # Safety timeout window scales down as speed step targets increase
        max_allowed_time = 20.0 if abs(rc_value) <= 10 else 8.0
        if time.time() - t0 > max_allowed_time:
            print(f"  [TIMEOUT] Target pad not found within {max_allowed_time}s safety limit.")
            break
            
        time.sleep(LOOP_DT)
        
    # --- PROPORTIONAL ACTIVE BRAKE ---
    # High speeds require a harsh counter-tilt to stop forward momentum over the pad
    brake_force = -int(np.sign(rc_value) * (abs(rc_value) * 0.65))
    brake_duration = 0.45 if abs(rc_value) > 30 else 0.25
    
    drone.send_rc_control(0, brake_force, 0, 0)
    time.sleep(brake_duration)
    drone.send_rc_control(0, 0, 0, 0)
    
    if detected and not danger_flag.is_set():
        actual_speed = PAD_DISTANCE_M / duration
        c_constant = actual_speed / abs(rc_value)
        print(f"    Success: Crossed in {duration:.2f}s | Speed: {actual_speed:.3f} m/s | C: {c_constant:.5f}")
        return duration, actual_speed, c_constant
        
    return None

def main():
    global is_airborne
    print("======================================================================")
    print(" HIGH-SPEED DUAL-PAD ODOMETRY CALIBRATOR (UP TO RC 50)")
    print(" Press [Ctrl+C] at any time to abort flight and force safe landing.")
    print("======================================================================")

    drone = Tello()
    drone.LOGGER.setLevel('ERROR')
    
    results = []
    
    try:
        drone.connect()
        print(f"Battery Charge: {drone.get_battery()}%")
        
        drone.enable_mission_pads()
        drone.set_mission_pad_detection_direction(0) # Downward looking camera only

        print("Launching drone into initial hover loop...")
        drone.takeoff()
        is_airborne = True
        time.sleep(2.0)

        # Post-Takeoff Pad Acquisition Phase
        print(f"Searching for initial tracking lock over Pad A (ID: {PAD_A_ID})...")
        pad_acquired = False
        search_start = time.time()
        
        while time.time() - search_start < 8.0 and not danger_flag.is_set():
            if drone.get_mission_pad_id() == PAD_A_ID:
                pad_acquired = True
                break
            drone.send_rc_control(0, 0, 0, 0)
            time.sleep(0.2)

        if not pad_acquired:
            print(f"[ERROR] Could not safely identify Pad A (ID: {PAD_A_ID}) from hover altitude. Aborting.")
            return

        # Perform the requested core center-alignment initialization
        stabilize_over_pad(drone, PAD_A_ID)

        # Iterate through the speed calibration steps up to 50
        for rc_speed in RC_SPEED_STEPS:
            if danger_flag.is_set(): break
            print(f"\n>>> COMMENCING TESTING MATRIX FOR RC VALUE: {rc_speed} <<<")
            
            for trial in range(1, TRIALS_PER_STEP + 1):
                if danger_flag.is_set(): break
                print(f"\nTrial {trial} of {TRIALS_PER_STEP} for RC {rc_speed}:")
                
                # --- FORWARD RUN (Pad A -> Pad B) ---
                stabilize_over_pad(drone, PAD_A_ID)  # Always re-center before step pulse
                fwd_data = execute_transit(drone, rc_speed, PAD_A_ID, PAD_B_ID, "FORWARD")
                if fwd_data:
                    results.append({"rc": rc_speed, "dir": "Forward", "time": fwd_data[0], "speed": fwd_data[1], "c": fwd_data[2]})
                
                # --- BACKWARD RUN (Pad B -> Pad A) ---
                stabilize_over_pad(drone, PAD_B_ID)  # Always re-center before step pulse
                bwd_data = execute_transit(drone, -rc_speed, PAD_B_ID, PAD_A_ID, "BACKWARD")
                if bwd_data:
                    results.append({"rc": rc_speed, "dir": "Backward", "time": bwd_data[0], "speed": bwd_data[1], "c": bwd_data[2]})

    except Exception as e:
        print(f"\nSystem Execution Failure: {e}")
    finally:
        print("\nTerminating profiles, engaging safe grounding protocol...")
        drone.send_rc_control(0, 0, 0, 0)
        time.sleep(0.2)
        
        if is_airborne:
            try:
                drone.land()
            except Exception as land_err:
                print(f"Landing command status: {land_err}")
        else:
            print("Drone already resting safely on surface. Landing request bypassed.")
            
        drone.disable_mission_pads()
        print("System shutdown sequence complete.")
        
    # Output metrics calculation summary table
    if len(results) > 0:
        print("\n\n" + "="*21 + " CALIBRATION PROFILE RESULTS " + "="*21)
        print(f"{'RC CMD':<8} | {'DIRECTION':<10} | {'DURATION':<9} | {'TRUE SPEED':<12} | {'CONSTANT (C)':<12}")
        print("-"*65)
        
        speed_groups = {speed: [] for speed in RC_SPEED_STEPS}
        
        for r in results:
            print(f"RC {r['rc']:<5} | {r['dir']:<10} | {r['time']:>6.2f}s    | {r['speed']:>7.3f} m/s | {r['c']:>10.5f}")
            speed_groups[r['rc']].append(r['c'])
            
        print("="*65)
        print("\nDerived Scaling Properties for Dead Reckoning:")
        for speed in RC_SPEED_STEPS:
            c_list = speed_groups[speed]
            if c_list:
                print(f"  Average Constant at RC {speed}: {np.mean(c_list):.5f} m/s per 1% RC")
            else:
                print(f"  Average Constant at RC {speed}: No valid data collected")
        print("="*65)
    else:
        print("\nNo full transit samples collected. Profile log empty.")

if __name__ == "__main__":
    main()