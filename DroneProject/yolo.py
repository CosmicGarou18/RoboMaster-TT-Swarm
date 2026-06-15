from djitellopy import Tello
from ultralytics import YOLO
import cv2
import time
import torch
import threading
import pygame
from collections import deque
from copy import deepcopy

class TelloYOLODetector:
    def __init__(self):
        # Initialize Tello drone
        self.tello = Tello()
        
        # Load YOLO model
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print(f"Using device: {self.device}")
        print("Loading YOLO model...")
        self.model = YOLO("yolo11n.pt")
        self.model.to(self.device)
        print("✅ YOLO model loaded!")
        
        # Flags for control
        self.is_flying = False
        self.running = True
        
        # RC control values (thread-safe)
        self.rc_left_right = 0
        self.rc_forward_back = 0
        self.rc_up_down = 0
        self.rc_yaw = 0
        self.rc_lock = threading.Lock()
        
        # Battery monitoring
        self.battery = 0
        self.last_battery_update = 0
        self.battery_update_interval = 5
        
        # FPS calculation
        self.fps = 0
        self.yolo_fps = 0
        self.fps_history = deque(maxlen=30)
        self.last_fps_time = time.time()
        self.frame_count = 0
        self.yolo_frame_count = 0
        self.last_yolo_fps_time = time.time()
        
        # Detection rate limiting for console
        self.last_detection_time = 0
        self.detection_cooldown = 0.3
        
        # Statistics
        self.detection_count = 0
        
        # Frame and detection storage (thread-safe)
        self.latest_frame = None
        self.latest_detections = []  # Stores boxes to draw
        self.frame_lock = threading.Lock()
        self.detection_lock = threading.Lock()
        
        # YOLO processing time tracking
        self.yolo_time = 0
        
        # Control for YOLO thread
        self.new_detection_available = False
        self.last_processed_frame = None
        
    def connect_and_start(self):
        """Connect to drone and start video stream"""
        print("Connecting to Tello...")
        self.tello.connect()
        self.update_battery()
        print(f"Battery: {self.battery}%")
        
        print("Starting video stream...")
        self.tello.streamon()
        
        self.frame_read = self.tello.get_frame_read()
        time.sleep(2)
        print("Ready for detection!")
        
    def update_battery(self):
        """Update battery percentage (rate limited)"""
        current_time = time.time()
        if current_time - self.last_battery_update > self.battery_update_interval:
            try:
                self.battery = self.tello.get_battery()
                self.last_battery_update = current_time
            except:
                pass
        return self.battery
    
    def calculate_fps(self):
        """Calculate display FPS"""
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_fps_time >= 1.0:
            self.fps = self.frame_count
            self.fps_history.append(self.fps)
            self.frame_count = 0
            self.last_fps_time = current_time
        return self.fps
    
    def calculate_yolo_fps(self):
        """Calculate YOLO processing FPS"""
        self.yolo_frame_count += 1
        current_time = time.time()
        if current_time - self.last_yolo_fps_time >= 1.0:
            self.yolo_fps = self.yolo_frame_count
            self.yolo_frame_count = 0
            self.last_yolo_fps_time = current_time
        return self.yolo_fps
    
    def fix_colors(self, frame):
        """Convert BGR to RGB for correct colors"""
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    def draw_detections(self, frame, detections):
        """Draw bounding boxes from cached detections"""
        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            class_name = detection['name']
            confidence = detection['confidence']
            
            # Draw bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{class_name}: {confidence:.2f}"
            
            # Add background for text
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            cv2.rectangle(frame, (x1, y1 - label_size[1] - 10), 
                        (x1 + label_size[0], y1), (0, 255, 0), -1)
            cv2.putText(frame, label, (x1, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        return frame
    
    def process_frame_yolo(self, frame):
        """Run YOLO detection on a single frame (runs in separate thread)"""
        start_time = time.time()
        
        # Run inference
        results = self.model(frame, stream=True, verbose=False)
        
        detections = []
        current_time = time.time()
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    cls_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    class_name = result.names[cls_id]
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    
                    detections.append({
                        'name': class_name,
                        'confidence': confidence,
                        'bbox': (x1, y1, x2, y2)
                    })
                    
                    # Rate-limited console output
                    if current_time - self.last_detection_time > self.detection_cooldown:
                        print(f"🔍 {class_name.upper()} ({confidence:.2f})")
                        self.last_detection_time = current_time
                        self.detection_count += 1
        
        # Calculate processing time
        self.yolo_time = (time.time() - start_time) * 1000  # ms
        self.calculate_yolo_fps()
        
        return detections
    
    def add_info_overlay(self, frame):
        """Add information overlay to frame"""
        self.update_battery()
        
        # Add FPS
        fps_text = f"Display FPS: {self.fps}"
        cv2.putText(frame, fps_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Add YOLO FPS
        yolo_fps_text = f"YOLO FPS: {self.yolo_fps} ({self.yolo_time:.0f}ms)"
        cv2.putText(frame, yolo_fps_text, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Add battery percentage
        battery_color = (0, 255, 0) if self.battery > 20 else (0, 0, 255)
        cv2.putText(frame, f"Battery: {self.battery}%", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, battery_color, 2)
        
        # Add flight status
        status_text = "FLYING" if self.is_flying else "GROUND"
        status_color = (0, 255, 0) if self.is_flying else (0, 0, 255)
        cv2.putText(frame, f"Status: {status_text}", (10, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        
        # Show RC control values
        with self.rc_lock:
            rc_text = f"RC: L/R={self.rc_left_right:3d} F/B={self.rc_forward_back:3d} U/D={self.rc_up_down:3d} Yaw={self.rc_yaw:3d}"
        cv2.putText(frame, rc_text, (10, 150), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Detection stats
        with self.detection_lock:
            detection_count = self.detection_count
        cv2.putText(frame, f"Total Detections: {detection_count}", (10, 180), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Cached detections indicator
        with self.detection_lock:
            has_detections = len(self.latest_detections) > 0
        if has_detections:
            cv2.putText(frame, "CACHED DETECTIONS ACTIVE", (10, 210), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Instructions
        instructions = "ESC=Quit | T=Takeoff | L=Land | WASD=Move | QE=Rotate | RF=Up/Down"
        cv2.putText(frame, instructions, (10, frame.shape[0] - 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Active controls indicator
        with self.rc_lock:
            active = []
            if self.rc_forward_back > 0: active.append("FORWARD")
            if self.rc_forward_back < 0: active.append("BACK")
            if self.rc_left_right > 0: active.append("RIGHT")
            if self.rc_left_right < 0: active.append("LEFT")
            if self.rc_up_down > 0: active.append("UP")
            if self.rc_up_down < 0: active.append("DOWN")
            if self.rc_yaw > 0: active.append("ROT R")
            if self.rc_yaw < 0: active.append("ROT L")
        
        if active:
            active_text = f"MOVING: {' '.join(active)}"
            cv2.putText(frame, active_text, (10, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        return frame
    
    def video_capture_loop(self):
        """Thread 1: Capture video from drone continuously"""
        while self.running:
            try:
                frame = self.frame_read.frame
                if frame is not None:
                    with self.frame_lock:
                        self.latest_frame = frame.copy()
                else:
                    time.sleep(0.005)
            except Exception as e:
                print(f"Video capture error: {e}")
                time.sleep(0.1)
    
    def yolo_processing_loop(self):
        """Thread 2: Run YOLO detection on frames (as fast as possible)"""
        last_processed_frame = None
        
        while self.running:
            # Get the latest frame
            with self.frame_lock:
                if self.latest_frame is None:
                    time.sleep(0.01)
                    continue
                current_frame = self.latest_frame.copy()
            
            # Only process if this is a new frame
            if last_processed_frame is not current_frame:
                # Run YOLO detection
                detections = self.process_frame_yolo(current_frame)
                
                # Store detections for display thread
                with self.detection_lock:
                    self.latest_detections = detections
                    self.new_detection_available = True
                
                last_processed_frame = current_frame
            
            # Small delay to prevent CPU overload
            time.sleep(0.005)
    
    def rc_control_loop(self):
        """Thread 3: Continuous RC control"""
        last_rc_update = 0
        while self.running:
            if self.is_flying:
                current_time = time.time()
                if current_time - last_rc_update > 0.05:  # 20Hz
                    with self.rc_lock:
                        self.tello.send_rc_control(
                            self.rc_left_right,
                            self.rc_forward_back,
                            self.rc_up_down,
                            self.rc_yaw
                        )
                    last_rc_update = current_time
            time.sleep(0.01)
    
    def keyboard_listener(self):
        """Thread 4: Pygame keyboard handling"""
        pygame.init()
        pygame.display.set_mode((100, 100))
        pygame.display.set_caption("Tello Control")
        
        print("\n" + "="*55)
        print("CONTROLS ACTIVE (Press and Hold for continuous movement):")
        print("="*55)
        print("  W         - Move Forward")
        print("  S         - Move Back")
        print("  A         - Move Left")
        print("  D         - Move Right")
        print("  R         - Move Up")
        print("  F         - Move Down")
        print("  Q         - Rotate Left")
        print("  E         - Rotate Right")
        print("  T         - Takeoff")
        print("  L         - Land")
        print("  SPACE     - Emergency Stop")
        print("  ESC       - Quit")
        print("="*55)
        print("🎨 YOLO runs in background, detections are CACHED and displayed")
        print("   This gives SMOOTH video with STABLE bounding boxes!\n")
        
        last_command_time = 0
        command_cooldown = 0.5
        
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                    break
            
            keys = pygame.key.get_pressed()
            
            with self.rc_lock:
                # Forward/Back (W/S)
                if keys[pygame.K_w]:
                    self.rc_forward_back = 50
                elif keys[pygame.K_s]:
                    self.rc_forward_back = -50
                else:
                    self.rc_forward_back = 0
                
                # Left/Right (A/D)
                if keys[pygame.K_a]:
                    self.rc_left_right = -50
                elif keys[pygame.K_d]:
                    self.rc_left_right = 50
                else:
                    self.rc_left_right = 0
                
                # Up/Down (R/F)
                if keys[pygame.K_r]:
                    self.rc_up_down = 50
                elif keys[pygame.K_f]:
                    self.rc_up_down = -50
                else:
                    self.rc_up_down = 0
                
                # Rotation (Q/E)
                if keys[pygame.K_q]:
                    self.rc_yaw = -50
                elif keys[pygame.K_e]:
                    self.rc_yaw = 50
                else:
                    self.rc_yaw = 0
            
            current_time = time.time()
            
            # Takeoff (T)
            if keys[pygame.K_t] and current_time - last_command_time > command_cooldown:
                if not self.is_flying:
                    print("🚁 Taking off...")
                    try:
                        self.tello.takeoff()
                        self.is_flying = True
                        print("✅ Takeoff complete!")
                    except Exception as e:
                        print(f"❌ Takeoff failed: {e}")
                else:
                    print("⚠️ Already flying!")
                last_command_time = current_time
            
            # Land (L)
            if keys[pygame.K_l] and current_time - last_command_time > command_cooldown:
                if self.is_flying:
                    print("🛬 Landing...")
                    try:
                        self.tello.land()
                        self.is_flying = False
                        with self.rc_lock:
                            self.rc_left_right = 0
                            self.rc_forward_back = 0
                            self.rc_up_down = 0
                            self.rc_yaw = 0
                        print("✅ Landing complete!")
                    except Exception as e:
                        print(f"❌ Landing failed: {e}")
                else:
                    print("⚠️ Not flying yet!")
                last_command_time = current_time
            
            # Emergency stop (SPACE)
            if keys[pygame.K_SPACE] and current_time - last_command_time > command_cooldown:
                if self.is_flying:
                    print("🚨 EMERGENCY STOP! 🚨")
                    try:
                        self.tello.emergency()
                        self.is_flying = False
                        with self.rc_lock:
                            self.rc_left_right = 0
                            self.rc_forward_back = 0
                            self.rc_up_down = 0
                            self.rc_yaw = 0
                        print("✅ Emergency stop executed!")
                    except Exception as e:
                        print(f"❌ Emergency stop failed: {e}")
                else:
                    print("⚠️ Drone is on ground")
                last_command_time = current_time
            
            # Quit (ESC)
            if keys[pygame.K_ESCAPE]:
                print("👋 Quitting...")
                self.running = False
                break
            
            time.sleep(0.02)
        
        pygame.quit()
    
    def display_loop(self):
        """Main display thread - draws frames continuously using cached detections"""
        while self.running:
            with self.frame_lock:
                if self.latest_frame is None:
                    time.sleep(0.005)
                    continue
                # Get the latest raw frame
                frame = self.latest_frame.copy()
            
            # Get the latest cached detections (from YOLO thread)
            with self.detection_lock:
                detections = self.latest_detections.copy()
            
            # Draw detections on frame (using cached results)
            frame = self.draw_detections(frame, detections)
            
            # Convert BGR to RGB for correct colors
            display_frame = self.fix_colors(frame)
            
            # Add overlay
            display_frame = self.add_info_overlay(display_frame)
            
            # Calculate FPS
            self.calculate_fps()
            
            # Display
            cv2.imshow("Tello Drone - YOLO Detection", display_frame)
            
            # Check for quit
            if cv2.waitKey(1) & 0xFF == 27:  # ESC key
                self.running = False
                break
            
            # Small delay to prevent CPU overload
            time.sleep(0.005)
    
    def run_detection(self):
        """Main method to start all threads"""
        # Start all threads
        threads = [
            threading.Thread(target=self.video_capture_loop, name="VideoCapture", daemon=True),
            threading.Thread(target=self.yolo_processing_loop, name="YOLOProcessing", daemon=True),
            threading.Thread(target=self.rc_control_loop, name="RCControl", daemon=True),
            threading.Thread(target=self.keyboard_listener, name="KeyboardListener", daemon=True),
        ]
        
        for thread in threads:
            thread.start()
            print(f"✅ Started thread: {thread.name}")
        
        print("\n🚀 All threads running!\n")
        
        # Run display in main thread
        self.display_loop()
        self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("\n🧹 Cleaning up...")
        self.running = False
        
        with self.rc_lock:
            self.rc_left_right = 0
            self.rc_forward_back = 0
            self.rc_up_down = 0
            self.rc_yaw = 0
        
        try:
            self.tello.send_rc_control(0, 0, 0, 0)
        except:
            pass
        
        cv2.destroyAllWindows()
        
        try:
            self.tello.streamoff()
        except:
            pass
        
        if self.is_flying:
            try:
                print("🛬 Landing drone...")
                self.tello.land()
                time.sleep(2)
            except:
                pass
        
        try:
            self.tello.end()
        except:
            pass
        
        print(f"✅ Done! Total detections: {self.detection_count}")

if __name__ == "__main__":
    print("=" * 60)
    print("     Tello Drone with YOLO Object Detection")
    print("     (Cached Detection Architecture)")
    print("=" * 60)
    
    detector = TelloYOLODetector()
    try:
        detector.connect_and_start()
        detector.run_detection()
    except Exception as e:
        print(f"\n❌ Failed to connect to drone: {e}")
        print("\n📋 Troubleshooting:")
        print("  1. Is the drone powered on?")
        print("  2. Connected to drone Wi-Fi?")
        print("  3. IP address correct? (10.65.164.114)")
        print("  4. Is yolo11n.pt in the project folder?")
        print("  5. Install pygame: pip install pygame")