#include <RMTT_Libs.h>
#include <Wire.h>

RMTT_RGB tt_rgb;
RMTT_Matrix tt_matrix;
RMTT_TOF tt_tof;
RMTT_Protocol protocol;

// ========== PATTERNS ==========

uint8_t readyPattern[8] = {
  0b00111100, 0b01000010, 0b10100101, 0b10000001,
  0b10100101, 0b10011001, 0b01000010, 0b00111100
};

// Rotate pattern (clockwise spiral)
uint8_t rotatePattern[8] = {
  0b11111111, 0b10000001, 0b10111101, 0b10100101,
  0b10100101, 0b10111101, 0b10000001, 0b11111111
};

// Danger pattern (X mark)
uint8_t dangerPattern[8] = {
  0b10000001, 0b01000010, 0b00100100, 0b00011000,
  0b00011000, 0b00100100, 0b01000010, 0b10000001
};

// Hover pattern (propeller)
uint8_t hoverPattern[8] = {
  0b00011000, 0b00100100, 0b01000010, 0b10000001,
  0b01000010, 0b00100100, 0b00011000, 0b00000000
};

// ========== HELPER FUNCTIONS ==========

void showPattern(uint8_t pattern[8]) {
  for(int row = 0; row < 8; row++) {
    for(int col = 0; col < 8; col++) {
      tt_matrix.SetLEDPWM(col, row, (pattern[row] & (1 << (7 - col))) ? 255 : 0);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== ToF Sensor Test ===");
  Serial.println("Commands:");
  Serial.println("  >50cm  -> Rotate CW");
  Serial.println("  20-50cm -> Hover");
  Serial.println("  <20cm  -> Land");
  Serial.println("========================");
  
  Wire.begin(27, 26);
  
  tt_rgb.Init();
  tt_matrix.Init(127);
  tt_matrix.On();
  
  // Initialize ToF
  if(tt_tof.Init(true)) {
    Serial.println("✅ ToF sensor ready!");
    tt_tof.StartContinuous(0);  // Fastest mode
    tt_tof.SetMeasurementTimingBudget(20000);  // 20ms for faster response
  } else {
    Serial.println("❌ ToF sensor not found!");
    while(1) delay(1000);
  }
  
  // Initialize drone
  Serial1.begin(1000000, SERIAL_8N1, 23, 18);
  
  showPattern(readyPattern);
  tt_rgb.SetRGB(0, 255, 0);
  
  // Wait for button
  pinMode(34, INPUT_PULLUP);
  Serial.println("Press button to start...");
  while(digitalRead(34) == HIGH) delay(50);
  
  // Enter SDK and takeoff
  protocol.startUntilControl();
  
  // ========== SAFETY CHECK BEFORE TAKEOFF ==========
  while (1) {
   uint16_t preCheck = tt_tof.ReadRangeContinuousMillimeters();
   if(preCheck < 300 && preCheck > 0) {
     Serial.println("⚠️ Object too close! Not taking off!");
     showPattern(dangerPattern);
     tt_rgb.SetRGB(255, 0, 0);
   } else {
    break;
   }
  }
  
  protocol.TakeOff();
  delay(3000);
  Serial.println("🚁 Flying! Testing ToF sensor...");
}

void loop() {
  // ========== READ DISTANCE ==========
  uint16_t distance = tt_tof.ReadRangeContinuousMillimeters();
  
  if(tt_tof.TimeoutOccurred()) {
    Serial.println("⚠️ Sensor timeout!");
    return;
  }
  
  // ========== COMMAND LOGIC ==========
  
  // 1. CLOSE DISTANCE (< 20cm) -> LAND
  if(distance < 200 && distance > 0) {
    Serial.print("🛑 CLOSE! Distance: ");
    Serial.print(distance);
    Serial.println(" mm -> LANDING!");
    
    showPattern(dangerPattern);
    tt_rgb.SetRGB(255, 0, 0);  // Red
    protocol.Land();
    
    Serial.println("✅ Landed!");
    while(1) {
      delay(1000);  // Stop here
    }
  }
  
  // 2. MEDIUM DISTANCE (20-50cm) -> HOVER
  else if(distance >= 200 && distance < 500) {
    Serial.print("⏸️  MEDIUM! Distance: ");
    Serial.print(distance);
    Serial.println(" mm -> HOVERING");
    
    showPattern(hoverPattern);
    tt_rgb.SetRGB(255, 255, 0);  // Yellow
    protocol.SetRC(0, 0, 0, 0);  // Stop
  }
  
  // 3. SAFE DISTANCE (> 50cm) -> ROTATE CLOCKWISE
  else {
    Serial.print("🔄 SAFE! Distance: ");
    Serial.print(distance);
    Serial.println(" mm -> ROTATING CW");
    
    showPattern(rotatePattern);
    tt_rgb.SetRGB(0, 0, 255);  // Blue
    protocol.SetRC(0, 0, 0, 30);  // Rotate clockwise at 30% speed
  }
  
  delay(100);  // Read every 100ms
}