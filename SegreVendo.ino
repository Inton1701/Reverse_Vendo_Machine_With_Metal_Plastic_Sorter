#include "HX711.h"
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>  // Settings storage
#include <avr/wdt.h> // Device reset functionality

//=============== PIN DEFINITIONS ===============//
// Sensors
#define PLASTIC_SENSOR_PIN 2    // Material detection - plastic
#define METAL_SENSOR_PIN 3      // Material detection - metal
#define ULTRASONIC_PLASTIC_TRIG 4
#define ULTRASONIC_PLASTIC_ECHO 5
#define ULTRASONIC_METAL_TRIG 7
#define ULTRASONIC_METAL_ECHO 6

// LEDs and outputs
#define LED_ERROR 36            // Error indicator
#define LED_PLASTIC 38          // Plastic detected
#define LED_METAL 40            // Metal detected
#define LED_PLASTIC_FULL 42     // Plastic bin full
#define LED_METAL_FULL 44       // Metal bin full
#define BUZZER 24               // Sound feedback
#define RELAY_PUMP 22           // Water pump control

// Scale
#define LOADCELL_DOUT 28        // Load cell data
#define LOADCELL_CLK 26         // Load cell clock

// Servos
#define SERVO1_PIN 30           // Main gate
#define SERVO3_PIN 34           // Water dispenser guide

// Controls
#define WATER_BUTTON 0          // Water dispense button
#define JOYSTICK_SW 46          // Joystick button
#define JOYSTICK_Y A6           // Y-axis (unused)
#define JOYSTICK_X A7           // X-axis (menu navigation)

// Settings storage
#define EEPROM_SETTINGS_ADDR 0  // EEPROM start address
#define SETTINGS_INITIALIZED_FLAG 0xAB  // Valid settings flag

//=============== COMPONENT INITIALIZATION ===============//
HX711 scale(LOADCELL_DOUT, LOADCELL_CLK);
Servo servo1;  // Main gate
Servo servo3;  // Water dispenser guide
LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C LCD

//=============== STATE VARIABLES ===============//
// System states
bool bottleOnScale = false;           // Bottle detected
bool bottleWeightProcessed = false;   // Weight validation done
bool gate1Open = false;               // Main gate open
bool plasticDetected = false;         // Plastic material found
bool metalDetected = false;           // Metal material found
bool materialDetectionActive = false; // Detection in progress
bool waterDispensing = false;         // Water dispensing active
bool inSettingsMode = false;          // Settings menu active
int currentMenuIndex = 0;             // Selected menu item

// Timing variables
unsigned long bottleDetectedTime = 0;
unsigned long gate1OpenTime = 0;
unsigned long lastWeightReading = 0;
unsigned long lastInvalidBuzzTime = 0;
unsigned long materialDetectionStartTime = 0;
unsigned long lastUltrasonicReading = 0;
unsigned long lastLCDUpdate = 0;
unsigned long waterDispenseStartTime = 0;
unsigned long lastButtonPress = 0;
unsigned long lastJoystickMove = 0;

// Sensor detection states
bool plasticPreviouslyDetected = false;
bool metalPreviouslyDetected = false;
unsigned long lastDetectionTime = 0;
unsigned long lastBuzzTime = 0;
unsigned long ledOnTime = 0;
bool ledActive = false;

// Bin monitoring
float plasticBinDistance = 0;
float metalBinDistance = 0;
bool plasticBinFull = false;
bool metalBinFull = false;
bool lastSortedWasPlastic = false;

// Servo positions
const int SERVO1_CLOSED = 0;
const int SERVO1_OPEN = 180;
const int SERVO3_NEUTRAL = 90;
const int SERVO3_PLASTIC = 45;
const int SERVO3_METAL = 135;

// Timing constants (milliseconds)
const unsigned long DETECTION_DELAY = 500;
const unsigned long CONFIRMATION_DELAY = 300;
const unsigned long BUZZ_COOLDOWN = 1000;
const unsigned long LED_DISPLAY_TIME = 2000;
const unsigned long SERVO_MOVE_DELAY = 200;  // Reduced from 1000 to 200
const unsigned long GATE_HOLD_TIME = 1000;   // Reduced from 2000 to 1000
const unsigned long ULTRASONIC_READING_INTERVAL = 500;
const unsigned long LCD_UPDATE_INTERVAL = 1000;
unsigned long WATER_DISPENSE_DURATION = 3000;  // Variable (calculated from flow rate)
const unsigned long BUTTON_DEBOUNCE_TIME = 500;
const unsigned long JOYSTICK_DEBOUNCE = 200;  // Menu navigation debounce

// System flags
bool invalidWeightDetected = false;
bool waitingForMaterialDetection = false;
bool sensorCooldownActive = false;
unsigned long sensorCooldownStart = 0;

// System timing
const unsigned long WEIGHT_READING_INTERVAL = 200;
const unsigned long WEIGHT_VALIDATION_DELAY = 2000;
const unsigned long GATE_OPEN_DURATION = 3000;
const unsigned long INVALID_BUZZ_INTERVAL = 500;
const unsigned long MATERIAL_DETECTION_TIMEOUT = 8000;
const unsigned long SENSOR_COOLDOWN_TIME = 2000;

// Weight thresholds (configurable)
float MIN_PLASTIC_WEIGHT = 13.0;      // Minimum plastic bottle weight
float MAX_PLASTIC_WEIGHT = 25.0;      // Maximum plastic bottle weight
float MIN_METAL_WEIGHT = 20.0;        // Minimum metal bottle weight
float MAX_METAL_WEIGHT = 35.0;        // Maximum metal bottle weight
const float BOTTLE_DETECTION_THRESHOLD = 10.0; // Scale detection threshold
float PLASTIC_FULL_AMOUNT = 10.0;      // Plastic bin full distance
float METAL_FULL_AMOUNT = 10.0;        // Metal bin full distance

// Calibration settings (configurable)
bool USE_MANUAL_CALIBRATION = true;  
float MANUAL_CALIBRATION_FACTOR = -243.0;

// Auto-calibration variables
float calibration_factor = 0;
bool calibration_complete = false;
int stable_readings = 0;
const int required_stable_readings = 3;
const float tolerance = 0.05;
unsigned long last_reading_time = 0;
const unsigned long reading_interval = 5;

// Credit and water system
int credits = 0;                      // Bottle credits earned
const int WATER_CREDIT_COST = 1;      // Credits per water dispense
float WATER_FLOW_RATE = 250.0;        // ml per minute
float WATER_AMOUNT = 250.0;           // ml to dispense

// Joystick input
int joystickXValue = 512;
int joystickYValue = 512;
bool joystickButtonPressed = false;
bool joystickButtonPrevious = false;

// Settings menu structure
const int NUM_MENU_ITEMS = 20;
const char* menuItems[NUM_MENU_ITEMS] = {
  "Run calibration",  // Scale calibration (auto/manual)
  "Cal mode",
  "Cal point", 
  "Run water cal",    // Water flow rate calibration
  "Flowrate(ml/min)",
  "Water amount",
  "Dispense water",
  "Min plastic wt",
  "Max plastic wt",
  "Min metal wt",
  "Max metal wt",
  "Plastic full",
  "Metal full",
  "Servo 1 test",
  "Servo 3 test",
  "Exit & Save",
  "Restore Default",
  "Reset Device"
};

// EEPROM settings structure
struct SystemSettings {
  byte initialized;  // Validity flag
  float flowRate;
  float waterAmount;
  bool useManualCalibration;
  float manualCalibrationFactor;
  float autoCalibrationFactor;
  float minPlasticWeight;
  float maxPlasticWeight;
  float minMetalWeight;
  float maxMetalWeight;
  float plasticFullAmount;
  float metalFullAmount;
};

SystemSettings settings;

void setup() {
  Serial.begin(9600);

  // Initialize sensor pins
  pinMode(PLASTIC_SENSOR_PIN, INPUT);
  pinMode(METAL_SENSOR_PIN, INPUT_PULLUP);
  pinMode(ULTRASONIC_PLASTIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_PLASTIC_ECHO, INPUT);
  pinMode(ULTRASONIC_METAL_TRIG, OUTPUT);
  pinMode(ULTRASONIC_METAL_ECHO, INPUT);
  pinMode(WATER_BUTTON, INPUT_PULLUP);

  // Initialize output pins
  pinMode(LED_ERROR, OUTPUT);
  pinMode(LED_PLASTIC, OUTPUT);
  pinMode(LED_METAL, OUTPUT);
  pinMode(LED_PLASTIC_FULL, OUTPUT);
  pinMode(LED_METAL_FULL, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(RELAY_PUMP, OUTPUT);

  // Set initial output states
  digitalWrite(LED_ERROR, LOW);
  digitalWrite(LED_PLASTIC, LOW);
  digitalWrite(LED_METAL, LOW);
  digitalWrite(LED_PLASTIC_FULL, LOW);
  digitalWrite(LED_METAL_FULL, LOW);
  digitalWrite(BUZZER, LOW);
  digitalWrite(RELAY_PUMP, HIGH);  // Pump OFF (inverted logic)

  // Setup servos
  servo1.attach(SERVO1_PIN);
  servo3.attach(SERVO3_PIN);
  
  servo1.write(SERVO1_CLOSED);
  servo3.write(SERVO3_NEUTRAL);

  // Setup LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Please wait...");
  lcd.setCursor(0, 1);
  delay(2000);

  // Initialize scale
  scale.set_scale();
  scale.tare();
  
  // Load settings then setup calibration
  loadSettings();
  setupCalibration();
  
  // Setup joystick pins
  pinMode(JOYSTICK_SW, INPUT_PULLUP);
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  
  loadSettings();
}

//=============== CALIBRATION FUNCTIONS ===============//
// Setup load cell calibration
void setupCalibration() {
  if (USE_MANUAL_CALIBRATION) {
    // Use manual calibration factor
    calibration_factor = MANUAL_CALIBRATION_FACTOR;
    scale.set_scale(calibration_factor);
    calibration_complete = true;
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Manual Cal Set");
    lcd.setCursor(0, 1);
    lcd.print("Factor: ");
    lcd.print((int)calibration_factor);
    delay(2000);
    
    float test_reading = scale.get_units(5);
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Test Reading:");
    lcd.setCursor(0, 1);
    lcd.print(test_reading, 2);
    lcd.print("g");
    delay(2000);
    
    Serial.println("System ready");
    buzzTwiceFast();
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Ready!");
    lcd.setCursor(0, 1);
    lcd.print("Place bottle");
    
  } else {
    // Run fresh auto calibration
    performAutoCalibration();
  }
}

//=============== MAIN LOOP ===============//
// Main program loop
void loop() {
  unsigned long currentTime = millis();
  
  readJoystick();
  
  // Handle settings menu
  if (inSettingsMode) {
    handleSettingsMenu(currentTime);
    return;
  }
  
  // Enter settings on joystick button
  if (joystickButtonPressed && !joystickButtonPrevious) {
    enterSettingsMode();
  }
  joystickButtonPrevious = joystickButtonPressed;

  // Continue calibration if needed
  if (!calibration_complete) {
    continueCalibration(currentTime);
    return;
  }

  // Skip all bottle processing when water is dispensing
  if (waterDispensing) {
    // Water button handling
    handleWaterButton(currentTime);
    
    // Water dispensing timeout
    if (currentTime - waterDispenseStartTime >= WATER_DISPENSE_DURATION) {
      stopWaterDispense();
    }
    
    // Display updates
    if (currentTime - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
      updateLCD();
      lastLCDUpdate = currentTime;
    }
    
    delay(10);
    return;
  }

  // Handle invalid weight warning
  if (invalidWeightDetected) {
    handleInvalidWeight(currentTime);
  }

  // Regular weight readings
  if (currentTime - lastWeightReading >= WEIGHT_READING_INTERVAL) {
    checkBottleWeight(currentTime);
    lastWeightReading = currentTime;
  }

  // Gate management
  manageGates(currentTime);

  // Sensor cooldown management
  if (sensorCooldownActive && (currentTime - sensorCooldownStart >= SENSOR_COOLDOWN_TIME)) {
    sensorCooldownActive = false;
  }

  // Bin level monitoring
  if (currentTime - lastUltrasonicReading >= ULTRASONIC_READING_INTERVAL) {
    readUltrasonicSensors();
    checkBinFullStatus();
    lastUltrasonicReading = currentTime;
  }

  // Display updates
  if (currentTime - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
    updateLCD();
    lastLCDUpdate = currentTime;
  }

  // Water button handling
  handleWaterButton(currentTime);

  // Material detection timeout
  if (waitingForMaterialDetection && (currentTime - materialDetectionStartTime >= MATERIAL_DETECTION_TIMEOUT)) {
    resetSystemState();
  }

  delay(10);
}

//=============== SENSOR FUNCTIONS ===============//
// Read bin level sensors
void readUltrasonicSensors() {
  // Plastic bin distance
  digitalWrite(ULTRASONIC_PLASTIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_PLASTIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_PLASTIC_TRIG, LOW);
  
  long duration1 = pulseIn(ULTRASONIC_PLASTIC_ECHO, HIGH, 30000);
  plasticBinDistance = (duration1 * 0.034) / 2;
  
  delay(50);
  
  // Metal bin distance
  digitalWrite(ULTRASONIC_METAL_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_METAL_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_METAL_TRIG, LOW);
  
  long duration2 = pulseIn(ULTRASONIC_METAL_ECHO, HIGH, 30000);
  metalBinDistance = (duration2 * 0.034) / 2;
}

// Check bin full status
void checkBinFullStatus() {
  // Plastic bin check
  if (plasticBinDistance <= PLASTIC_FULL_AMOUNT) {
    if (!plasticBinFull) {
      plasticBinFull = true;
      digitalWrite(LED_PLASTIC_FULL, HIGH);
      buzzWarning();
    }
  } else {
    if (plasticBinFull) {
      plasticBinFull = false;
      digitalWrite(LED_PLASTIC_FULL, LOW);
    }
  }
  
  // Metal bin check
  if (metalBinDistance <= METAL_FULL_AMOUNT) {
    if (!metalBinFull) {
      metalBinFull = true;
      digitalWrite(LED_METAL_FULL, HIGH);
      buzzWarning();
    }
  } else {
    if (metalBinFull) {
      metalBinFull = false;
      digitalWrite(LED_METAL_FULL, LOW);
    }
  }
}

// Validate bottle weight and material detection
void validateBottleWeight(unsigned long currentTime, float weight) {
  // Read material sensors
  bool rawPlastic = digitalRead(PLASTIC_SENSOR_PIN) == HIGH;
  bool rawMetal = digitalRead(METAL_SENSOR_PIN) == LOW;
  bool isPlastic = rawPlastic && !rawMetal;
  bool isMetal = rawMetal;
  
  bool validWeight = false;
  
  // Check weight based on material type
  if (isPlastic) {
    validWeight = (weight >= MIN_PLASTIC_WEIGHT && weight <= MAX_PLASTIC_WEIGHT);
  } else if (isMetal) {
    validWeight = (weight >= MIN_METAL_WEIGHT && weight <= MAX_METAL_WEIGHT);
  }
  
  // Check if both weight and material detection are valid
  if (validWeight && (isPlastic || isMetal)) {
    // Check bin capacity
    if ((isPlastic && plasticBinFull) || (isMetal && metalBinFull)) {
      digitalWrite(LED_ERROR, HIGH);
      invalidWeightDetected = true;
      lastInvalidBuzzTime = currentTime;
      buzzWarning();
      return;
    }
    
    // Set material flags
    plasticDetected = isPlastic;
    metalDetected = isMetal;
    
    // Position servo3 before opening gate based on material type
    if (isPlastic) {
      servo3.write(SERVO3_PLASTIC);  // Guide to plastic bin
    } else if (isMetal) {
      servo3.write(SERVO3_METAL);    // Guide to metal bin
    }
    delay(SERVO_MOVE_DELAY);  // Wait for servo to reach position
    
    buzzTwiceFast();
    openGate1(currentTime);
  } else {
    digitalWrite(LED_ERROR, HIGH);
    invalidWeightDetected = true;
    lastInvalidBuzzTime = currentTime;
    buzzWarning();
  }
}

// Check bottle weight
void checkBottleWeight(unsigned long currentTime) {
  float weight = scale.get_units(3);
  if (weight < 0) weight = 0.00;

  // Detect bottle placement
  if (weight >= BOTTLE_DETECTION_THRESHOLD) {
    if (!bottleOnScale) {
      // New bottle detected
      bottleOnScale = true;
      bottleDetectedTime = currentTime;
      bottleWeightProcessed = false;
    }

    // Validate weight after delay
    if (!bottleWeightProcessed && (currentTime - bottleDetectedTime >= WEIGHT_VALIDATION_DELAY)) {
      validateBottleWeight(currentTime, weight);
      bottleWeightProcessed = true;
    }
  } else {
    // Bottle removed
    if (bottleOnScale) {
      bottleOnScale = false;
      bottleWeightProcessed = false;

      if (gate1Open) {
        // Bottle passed through gate - perform sorting
        if (plasticDetected) {
          performPlasticSorting();
        } else if (metalDetected) {
          performMetalSorting();
        }
        resetSystemState();
      } else {
        // Bottle removed without processing
        resetSystemState();
      }
    }
  }
}

// Manage gate timing
void manageGates(unsigned long currentTime) {
  // Auto-close main gate
  if (gate1Open && (currentTime - gate1OpenTime >= GATE_OPEN_DURATION)) {
    servo1.write(SERVO1_CLOSED);
    gate1Open = false;
  }
}

// Sort plastic bottle
void performPlasticSorting() {
  credits++;
  
  // Servo3 is already positioned, just wait then return to neutral
  delay(SERVO_MOVE_DELAY);  // Allow bottle to pass through
  servo3.write(SERVO3_NEUTRAL);  // Return to neutral position
  delay(SERVO_MOVE_DELAY);
}

// Sort metal bottle
void performMetalSorting() {
  credits++;
  
  // Servo3 is already positioned, just wait then return to neutral
  delay(SERVO_MOVE_DELAY);  // Allow bottle to pass through
  servo3.write(SERVO3_NEUTRAL);  // Return to neutral position
  delay(SERVO_MOVE_DELAY);
}

// Start water dispensing
void startWaterDispense(unsigned long currentTime) {
  waterDispensing = true;
  waterDispenseStartTime = currentTime;
  digitalWrite(RELAY_PUMP, LOW);  // Turn ON (inverted logic)
  buzzShort();
}

// Stop water dispensing
void stopWaterDispense() {
  waterDispensing = false;
  digitalWrite(RELAY_PUMP, HIGH);  // Turn OFF (inverted logic)
  buzzShort();
  servo3.write(SERVO3_NEUTRAL);
}

// Update LCD display
void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  
  if (!calibration_complete) {
    lcd.print("Calibrating...");
    return;
  }
  
  if (waterDispensing) {
    lcd.print("Water Dispensing");
    lcd.setCursor(0, 1);
    lcd.print("Please wait...");
    return;
  }
  
  if (inSettingsMode) {
    updateSettingsDisplay();
    return;
  }
  
  // Check for bin full alerts first
  bool rawPlastic = digitalRead(PLASTIC_SENSOR_PIN) == HIGH;
  bool rawMetal = digitalRead(METAL_SENSOR_PIN) == LOW;
  
  if (plasticBinFull && rawPlastic) {
    lcd.print("PLASTIC BIN FULL");
    lcd.setCursor(0, 1);
    lcd.print("EMPTY BIN!");
    return;
  }
  
  if (metalBinFull && rawMetal) {
    lcd.print("METAL BIN FULL");
    lcd.setCursor(0, 1);
    lcd.print("EMPTY BIN!");
    return;
  }
  
  if (bottleOnScale) {
    float weight = scale.get_units(3);
    if (weight < 0) weight = 0.00;
    lcd.print("Weight: ");
    lcd.print(weight, 1);
    lcd.print("g");
    lcd.setCursor(0, 1);
    if (invalidWeightDetected) {
      lcd.print("INVALID REMOVE");
    } else {
      lcd.print("Validating...");
    }
  } else if (waitingForMaterialDetection) {
    lcd.print("Detecting...");
    lcd.setCursor(0, 1);
    lcd.print("Material check");
  } else if (plasticDetected || metalDetected) {
    lcd.print("Sorting: ");
    lcd.print(plasticDetected ? "Plastic" : "Metal");
    lcd.setCursor(0, 1);
    lcd.print("Please wait...");
  } else {
    // Ready state with credits
    lcd.print("READY - CRED:");
    lcd.print(credits);
    lcd.setCursor(0, 1);
    lcd.print("Place a bottle");
  }
}

// Update settings menu display
void updateSettingsDisplay() {
  lcd.clear();
  
  // Show category headers
  if (currentMenuIndex < 3) {
    lcd.print("SCALE CAL");
  } else if (currentMenuIndex < 7) {
    lcd.print("DISPENSE CAL");
  } else if (currentMenuIndex < 13) {
    lcd.print("WEIGHT CAL");
  } else if (currentMenuIndex < 15) {
    lcd.print("SERVO TEST");
  } else {
    lcd.print("SYSTEM");
  }
  
  lcd.setCursor(0, 1);
  lcd.print("> ");
  
  // Special handling for calibration displays
  if (currentMenuIndex == 0) {
    // Scale calibration - show mode dependent
    if (settings.useManualCalibration) {
      lcd.print("Manual Cal");
    } else {
      lcd.print("Auto Cal");
    }
  } else if (currentMenuIndex == 3) {
    // Water calibration - always show Water Cal
    lcd.print("Water Cal");
  } else {
    lcd.print(menuItems[currentMenuIndex]);
  }
}

// Plastic detection feedback
void activatePlasticResponse(unsigned long currentTime) {
  digitalWrite(LED_PLASTIC, HIGH);
  digitalWrite(LED_METAL, LOW);
  digitalWrite(LED_ERROR, LOW);
  ledActive = true;
  ledOnTime = currentTime;
  
  if (currentTime - lastBuzzTime > BUZZ_COOLDOWN) {
    buzzShort();
    lastBuzzTime = currentTime;
  }
}

// Metal detection feedback
void activateMetalResponse(unsigned long currentTime) {
  digitalWrite(LED_METAL, HIGH);
  digitalWrite(LED_PLASTIC, LOW);
  digitalWrite(LED_ERROR, LOW);
  ledActive = true;
  ledOnTime = currentTime;
  
  if (currentTime - lastBuzzTime > BUZZ_COOLDOWN) {
    buzzShort();
    lastBuzzTime = currentTime;
  }
}

// Short buzz
void buzzShort() {
  digitalWrite(BUZZER, HIGH);
  delay(200);
  digitalWrite(BUZZER, LOW);
}

// Warning buzz pattern
void buzzWarning() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(100);
    digitalWrite(BUZZER, LOW);
    delay(100);
  }
}

// Start sensor cooldown
void startSensorCooldown(unsigned long currentTime) {
  sensorCooldownActive = true;
  sensorCooldownStart = currentTime;
}

// Reset system state
void resetSystemState() {
  // Turn off indicators
  digitalWrite(LED_PLASTIC, LOW);
  digitalWrite(LED_METAL, LOW);
  digitalWrite(LED_ERROR, LOW);
  digitalWrite(BUZZER, LOW);

  // Close gates
  if (gate1Open) {
    servo1.write(SERVO1_CLOSED);
    gate1Open = false;
  }
  
  // Reset servo positions
  servo3.write(SERVO3_NEUTRAL);

  // Reset flags
  bottleOnScale = false;
  bottleWeightProcessed = false;
  plasticDetected = false;
  metalDetected = false;
  invalidWeightDetected = false;
  waitingForMaterialDetection = false;
  materialDetectionActive = false;
  
  plasticPreviouslyDetected = false;
  metalPreviouslyDetected = false;
  ledActive = false;
}

// Handle invalid weight warning
void handleInvalidWeight(unsigned long currentTime) {
  // Periodic buzzing
  if (currentTime - lastInvalidBuzzTime >= INVALID_BUZZ_INTERVAL) {
    digitalWrite(BUZZER, HIGH);
    delay(100);
    digitalWrite(BUZZER, LOW);
    lastInvalidBuzzTime = currentTime;
  }
}

// Double buzz confirmation
void buzzTwiceFast() {
  digitalWrite(BUZZER, HIGH);
  delay(150);
  digitalWrite(BUZZER, LOW);
  delay(100);
  digitalWrite(BUZZER, HIGH);
  delay(150);
  digitalWrite(BUZZER, LOW);
}

//=============== AUTO CALIBRATION FUNCTIONS ===============//
// Start auto calibration
void performAutoCalibration() {
  calibration_factor = 0;
  calibration_complete = false;
  stable_readings = 0;
  last_reading_time = millis();
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibrating...");
  lcd.setCursor(0, 1);
  lcd.print("Please wait");
}

// Continue calibration process
void continueCalibration(unsigned long currentTime) {
  if (currentTime - last_reading_time >= reading_interval) {
    last_reading_time = currentTime;

    scale.set_scale(calibration_factor);
    float units = scale.get_units(1);
    
    // Show live progress
    lcd.setCursor(0, 0);
    lcd.print("Factor: ");
    lcd.print((int)calibration_factor);
    lcd.print("     ");
    
    lcd.setCursor(0, 1);
    lcd.print("Wt: ");
    lcd.print(abs(units), 2);
    lcd.print("g     ");

    if (abs(units) <= tolerance) {
      stable_readings++;
      if (stable_readings >= required_stable_readings) {
        calibration_complete = true;
        
        // Save calibration factor
        settings.autoCalibrationFactor = calibration_factor;
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("System Ready!");
        lcd.setCursor(0, 1);
        lcd.print("Place bottle");
        
        buzzTwiceFast();
      }
    } else {
      stable_readings = 0;
      calibration_factor -= 1;
    }
  }
}

// Manual auto calibration from settings
void runAutoCalibration() {
  lcd.clear();
  lcd.print("Auto Calibration");
  lcd.setCursor(0, 1);
  lcd.print("Press to start");
  
  // Wait for button press
  joystickButtonPrevious = true;
  delay(500);
  
  while (true) {
    readJoystick();
    if (joystickButtonPressed && !joystickButtonPrevious) {
      break;
    }
    joystickButtonPrevious = joystickButtonPressed;
    delay(10);
  }
  
  // Start calibration
  calibration_factor = 0;
  calibration_complete = false;
  stable_readings = 0;
  last_reading_time = millis();
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibrating...");
  lcd.setCursor(0, 1);
  lcd.print("Please wait");
  
  // Calibration loop
  while (!calibration_complete) {
    unsigned long currentTime = millis();
    
    if (currentTime - last_reading_time >= reading_interval) {
      last_reading_time = currentTime;

      scale.set_scale(calibration_factor);
      float units = scale.get_units(1);
      
      // Show live progress
      lcd.setCursor(0, 0);
      lcd.print("Factor: ");
      lcd.print((int)calibration_factor);
      lcd.print("    ");
      
      lcd.setCursor(0, 1);
      lcd.print("Wt: ");
      lcd.print(abs(units), 2);
      lcd.print("g    ");

      if (abs(units) <= tolerance) {
        stable_readings++;
        if (stable_readings >= required_stable_readings) {
          calibration_complete = true;
          
          // Save calibration factor
          settings.autoCalibrationFactor = calibration_factor;
          
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Calibration");
          lcd.setCursor(0, 1);
          lcd.print("Complete!");
          delay(2000);
          
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Factor: ");
          lcd.print((int)calibration_factor);
          delay(2000);
          
          buzzTwiceFast();
        }
      } else {
        stable_readings = 0;
        calibration_factor -= 1;
      }
    }
    
    delay(10);
  }
  
  updateSettingsDisplay();
}

// Read joystick input
void readJoystick() {
  joystickXValue = analogRead(JOYSTICK_X);
  // Y-axis disabled (set to center)
  joystickYValue = 512;
  joystickButtonPressed = (digitalRead(JOYSTICK_SW) == LOW);
}

// Enter settings menu
void enterSettingsMode() {
  inSettingsMode = true;
  currentMenuIndex = 0;
  updateSettingsDisplay();
  // Prevent immediate selection
  joystickButtonPrevious = true;
  delay(300);
}

// Handle settings menu navigation
void handleSettingsMenu(unsigned long currentTime) {
  if (currentTime - lastJoystickMove < JOYSTICK_DEBOUNCE) {
    return;
  }
  
  // X-axis navigation (left/right)
  if (joystickXValue < 300) {  // Left - previous item
    currentMenuIndex = (currentMenuIndex > 0) ? currentMenuIndex - 1 : NUM_MENU_ITEMS - 1;
    updateSettingsDisplay();
    lastJoystickMove = currentTime;
  }
  else if (joystickXValue > 700) {  // Right - next item
    currentMenuIndex = (currentMenuIndex < NUM_MENU_ITEMS - 1) ? currentMenuIndex + 1 : 0;
    updateSettingsDisplay();
    lastJoystickMove = currentTime;
  }
  
  // Button press for selection
  if (joystickButtonPressed && !joystickButtonPrevious) {
    
    if (currentMenuIndex == 15) {  // Exit & Save - updated index
      lcd.clear();
      lcd.print("Saving settings");
      lcd.setCursor(0, 1);
      lcd.print("and restarting...");
      delay(1000);
      saveSettings();
    } 
    else if (currentMenuIndex == 17) {  // Reset Device - updated index
      resetDevice();
    }
    else if (currentMenuIndex == 16) {  // Restore Default - updated index
      restoreDefaultSettings();
    }
    else if (currentMenuIndex == 13) {  // Servo 1 test
      testServo1();
    }
    else if (currentMenuIndex == 14) {  // Servo 3 test
      testServo3();
    }
    else {
      // Adjust setting value
      adjustSettingValue(currentMenuIndex, currentTime);
    }
    joystickButtonPrevious = true;
  } 
  else if (!joystickButtonPressed) {
    joystickButtonPrevious = false;
  }
}

// Adjust the value of the selected setting
void adjustSettingValue(int settingIndex, unsigned long currentTime) {
  bool adjusting = true;
  int tempValue = 0;
  
  // Get current value based on selected setting
  switch (settingIndex) {
    case 0:  // Run calibration - scale calibration (auto/manual)
      if (settings.useManualCalibration) {
        // Manual scale calibration - adjust calibration point
        displayCalibrationPoint();
      } else {
        // Auto scale calibration
        runAutoCalibration();
      }
      return;
    case 1:  // Cal mode
      toggleCalibrationMode();
      return;
    case 2:  // Cal point - mode dependent
      displayCalibrationPoint();
      return;
    case 3:  // Run water cal - water flow rate calibration
      runWaterCalibration();
      return;
    case 4:  // Flowrate
      tempValue = settings.flowRate;
      break;
    case 5:  // Water amount
      tempValue = settings.waterAmount;
      break;
    case 6:  // Dispense water - special case
      runManualWaterDispense();
      return;
    case 7:  // Min plastic weight
      tempValue = settings.minPlasticWeight * 10;
      break;
    case 8:  // Max plastic weight
      tempValue = settings.maxPlasticWeight * 10;
      break;
    case 9:  // Min metal weight
      tempValue = settings.minMetalWeight * 10;
      break;
    case 10:  // Max metal weight
      tempValue = settings.maxMetalWeight * 10;
      break;
    case 11:  // Plastic full - show live bin level and adjust threshold
      displayBinLevel(true); // true for plastic
      return;
    case 12:  // Metal full - show live bin level and adjust threshold
      displayBinLevel(false); // false for metal
      return;
  }
  // Regular value adjustment for other settings
  lcd.clear();
  lcd.print(menuItems[settingIndex]);
  lcd.setCursor(0, 1);
  lcd.print("Value: ");
  
  if (settingIndex == 4) {
    lcd.print(tempValue);  // Integer values for flowrate
  } else if (settingIndex == 5) {
    lcd.print(tempValue);
    lcd.print("ml");  // Show ml unit for water amount
  } else {
    lcd.print(tempValue / 10.0, 1);  // Decimal values
  }
  
  lastJoystickMove = currentTime;
  joystickButtonPrevious = true;
  delay(300);
  
  while (adjusting) {
    readJoystick();
    currentTime = millis();
    
    if (currentTime - lastJoystickMove >= JOYSTICK_DEBOUNCE) {
      if (joystickXValue < 300) {  // Left - decrease
        if (settingIndex == 5 && tempValue > 1) {  // Water amount - 1ml increments, minimum 1ml
          tempValue--;
        } else if (settingIndex != 5) {
          tempValue--;
        }
        lastJoystickMove = currentTime;
        lcd.setCursor(0, 1);
        lcd.print("Value: ");
        if (settingIndex == 4) {
          lcd.print(tempValue);
          lcd.print("    ");
        } else if (settingIndex == 5) {
          lcd.print(tempValue);
          lcd.print("ml    ");
        } else {
          lcd.print(tempValue / 10.0, 1);
          lcd.print("    ");
        }
      }
      else if (joystickXValue > 700) {  // Right - increase
        if (settingIndex == 5 && tempValue < 2000) {  // Water amount - 1ml increments, maximum 2000ml
          tempValue++;
        } else if (settingIndex != 5) {
          tempValue++;
        }
        lastJoystickMove = currentTime;
        lcd.setCursor(0, 1);
        lcd.print("Value: ");
        if (settingIndex == 4) {
          lcd.print(tempValue);
          lcd.print("    ");
        } else if (settingIndex == 5) {
          lcd.print(tempValue);
          lcd.print("ml    ");
        } else {
          lcd.print(tempValue / 10.0, 1);
          lcd.print("    ");
        }
      }
    }
    
    // Button press to confirm and exit
    if (joystickButtonPressed && !joystickButtonPrevious) {
      // Save the adjusted value
      switch (settingIndex) {
        case 4:  // Flowrate
          settings.flowRate = tempValue;
          WATER_FLOW_RATE = settings.flowRate;
          break;
        case 5:  // Water amount
          settings.waterAmount = tempValue;
          WATER_AMOUNT = settings.waterAmount;
          WATER_DISPENSE_DURATION = (WATER_AMOUNT / WATER_FLOW_RATE) * 60 * 1000;
          break;
        case 7:  // Min plastic weight
          settings.minPlasticWeight = tempValue / 10.0;
          MIN_PLASTIC_WEIGHT = settings.minPlasticWeight;
          break;
        case 8:  // Max plastic weight
          settings.maxPlasticWeight = tempValue / 10.0;
          MAX_PLASTIC_WEIGHT = settings.maxPlasticWeight;
          break;
        case 9:  // Min metal weight
          settings.minMetalWeight = tempValue / 10.0;
          MIN_METAL_WEIGHT = settings.minMetalWeight;
          break;
        case 10:  // Max metal weight
          settings.maxMetalWeight = tempValue / 10.0;
          MAX_METAL_WEIGHT = settings.maxMetalWeight;
          break;

      }
      
      adjusting = false;
      joystickButtonPrevious = true;
      delay(300);
      updateSettingsDisplay();
    }
    else if (!joystickButtonPressed) {
      joystickButtonPrevious = false;
    }
    
    delay(10);
  }
}

//=============== EEPROM FUNCTIONS ===============//
// Load settings from EEPROM
void loadSettings() {
  EEPROM.get(EEPROM_SETTINGS_ADDR, settings);
  
  // Check if settings are valid
  if (settings.initialized != SETTINGS_INITIALIZED_FLAG) {
    // Use defaults
    settings.initialized = SETTINGS_INITIALIZED_FLAG;
    settings.flowRate = WATER_FLOW_RATE;
    settings.waterAmount = WATER_AMOUNT;
    settings.useManualCalibration = USE_MANUAL_CALIBRATION;
    settings.manualCalibrationFactor = MANUAL_CALIBRATION_FACTOR;
    settings.autoCalibrationFactor = 0;
    settings.minPlasticWeight = MIN_PLASTIC_WEIGHT;
    settings.maxPlasticWeight = MAX_PLASTIC_WEIGHT;
    settings.minMetalWeight = MIN_METAL_WEIGHT;
    settings.maxMetalWeight = MAX_METAL_WEIGHT;
    settings.plasticFullAmount = PLASTIC_FULL_AMOUNT;
    settings.metalFullAmount = METAL_FULL_AMOUNT;
    
    saveSettings();
  } else {
    // Apply loaded settings
    WATER_FLOW_RATE = settings.flowRate;
    WATER_AMOUNT = settings.waterAmount;
    WATER_DISPENSE_DURATION = (WATER_AMOUNT / WATER_FLOW_RATE) * 60 * 1000;
    USE_MANUAL_CALIBRATION = settings.useManualCalibration;
    MANUAL_CALIBRATION_FACTOR = settings.manualCalibrationFactor;
    MIN_PLASTIC_WEIGHT = settings.minPlasticWeight;
    MAX_PLASTIC_WEIGHT = settings.maxPlasticWeight;
    MIN_METAL_WEIGHT = settings.minMetalWeight;
    MAX_METAL_WEIGHT = settings.maxMetalWeight;
    PLASTIC_FULL_AMOUNT = settings.plasticFullAmount;
    METAL_FULL_AMOUNT = settings.metalFullAmount;
  }
}

// Save settings to EEPROM and restart device
void saveSettings() {
  // Update settings structure
  settings.flowRate = WATER_FLOW_RATE;
  settings.waterAmount = WATER_AMOUNT;
  settings.useManualCalibration = USE_MANUAL_CALIBRATION;
  settings.manualCalibrationFactor = MANUAL_CALIBRATION_FACTOR;
  settings.autoCalibrationFactor = calibration_factor;
  settings.minPlasticWeight = MIN_PLASTIC_WEIGHT;
  settings.maxPlasticWeight = MAX_PLASTIC_WEIGHT;
  settings.minMetalWeight = MIN_METAL_WEIGHT;
  settings.maxMetalWeight = MAX_METAL_WEIGHT;
  settings.plasticFullAmount = PLASTIC_FULL_AMOUNT;
  settings.metalFullAmount = METAL_FULL_AMOUNT;
  
  // Save to EEPROM
  EEPROM.put(EEPROM_SETTINGS_ADDR, settings);
  
  lcd.clear();
  lcd.print("Settings saved!");
  lcd.setCursor(0, 1);
  lcd.print("Restarting...");
  
  delay(2000);
  
  // Enable watchdog timer to reset device
  wdt_enable(WDTO_15MS);
  while (1) {} // Wait for reset
}

// Reset Arduino using watchdog
void resetDevice() {
  lcd.clear();
  lcd.print("Resetting...");
  lcd.setCursor(0, 1);
  lcd.print("Please wait");
  
  saveSettings();
  delay(1500);
  
  // Enable watchdog timer
  wdt_enable(WDTO_15MS);
  
  // Wait for reset
  while(1) {}
}

// Restore factory defaults
void restoreDefaultSettings() {
  lcd.clear();
  lcd.print("Restore default?");
  lcd.setCursor(0, 1);
  lcd.print("Press to confirm");
  
  delay(1000);
  
  bool prevButtonState = joystickButtonPressed;
  unsigned long confirmTimeout = millis();
  
  while (millis() - confirmTimeout < 5000) {
    readJoystick();
    
    if (!joystickButtonPressed && prevButtonState) {
      prevButtonState = false;
    }
    else if (joystickButtonPressed && !prevButtonState) {
      // Set defaults
      WATER_FLOW_RATE = 250.0;
      WATER_AMOUNT = 250.0;
      USE_MANUAL_CALIBRATION = true;
      MANUAL_CALIBRATION_FACTOR = -243.0;
      MIN_PLASTIC_WEIGHT = 13.0;
      MAX_PLASTIC_WEIGHT = 25.0;
      MIN_METAL_WEIGHT = 20.0;
      MAX_METAL_WEIGHT = 35.0;
      PLASTIC_FULL_AMOUNT = 10.0;
      METAL_FULL_AMOUNT = 10.0;
      
      // Update runtime values
      calibration_factor = MANUAL_CALIBRATION_FACTOR;
      scale.set_scale(calibration_factor);
      calibration_complete = true;
      WATER_DISPENSE_DURATION = (WATER_AMOUNT / WATER_FLOW_RATE) * 60 * 1000;
      
      // Save to EEPROM
      settings.flowRate = WATER_FLOW_RATE;
      settings.waterAmount = WATER_AMOUNT;
      settings.useManualCalibration = USE_MANUAL_CALIBRATION;
      settings.manualCalibrationFactor = MANUAL_CALIBRATION_FACTOR;
      settings.autoCalibrationFactor = 0;
      settings.minPlasticWeight = MIN_PLASTIC_WEIGHT;
      settings.maxPlasticWeight = MAX_PLASTIC_WEIGHT;
      settings.minMetalWeight = MIN_METAL_WEIGHT;
      settings.maxMetalWeight = MAX_METAL_WEIGHT;
      settings.plasticFullAmount = PLASTIC_FULL_AMOUNT;
      settings.metalFullAmount = METAL_FULL_AMOUNT;
      EEPROM.put(EEPROM_SETTINGS_ADDR, settings);
      
      lcd.clear();
      lcd.print("Defaults");
      lcd.setCursor(0, 1);
      lcd.print("Restored!");
      delay(1500);
      
      lcd.clear();
      lcd.print("Restarting...");
      lcd.setCursor(0, 1);
      lcd.print("Please wait");
      delay(1000);
      
      // Enable watchdog timer to reset device
      wdt_enable(WDTO_15MS);
      while (1) {} // Wait for reset
    }
    
    delay(10);
  }
  
  lcd.clear();
  lcd.print("Cancelled");
  delay(1000);
  updateSettingsDisplay();
}

// Run water calibration for 30 seconds to determine flow rate
void runWaterCalibration() {
  lcd.clear();
  lcd.print("Water Cal");
  lcd.setCursor(0, 1);
  lcd.print("Press to start");
  
  // Wait for button press
  joystickButtonPrevious = true;
  delay(500);
  
  while (true) {
    readJoystick();
    if (joystickButtonPressed && !joystickButtonPrevious) {
      break;
    }
    joystickButtonPrevious = joystickButtonPressed;
    delay(10);
  }
  
  lcd.clear();
  lcd.print("Running for 30s");
  
  // Run pump for exactly 30 seconds
  digitalWrite(RELAY_PUMP, LOW);  // Turn ON pump
  delay(30000);  // Wait 30 seconds
  digitalWrite(RELAY_PUMP, HIGH); // Turn OFF pump
  
  lcd.clear();
  lcd.print("Enter amount(ml)");
  lcd.setCursor(0, 1);
  lcd.print("Value: 0");
  
  // Input measured amount
  int measuredAmount = 0;
  joystickButtonPrevious = true;
  delay(300);
  
  while (true) {
    readJoystick();
    unsigned long currentTime = millis();
    
    if (currentTime - lastJoystickMove >= JOYSTICK_DEBOUNCE) {
      if (joystickXValue < 300) {  // Left - decrease
        if (measuredAmount > 0) measuredAmount--;
        lastJoystickMove = currentTime;
        lcd.setCursor(0, 1);
        lcd.print("Value: ");
        lcd.print(measuredAmount);
        lcd.print("    ");
      }
      else if (joystickXValue > 700) {  // Right - increase
        measuredAmount++;
        lastJoystickMove = currentTime;
        lcd.setCursor(0, 1);
        lcd.print("Value: ");
        lcd.print(measuredAmount);
        lcd.print("    ");
      }
    }
    
    if (joystickButtonPressed && !joystickButtonPrevious) {
      // Calculate flow rate (ml per minute) from 30-second measurement
      settings.flowRate = (measuredAmount * 60.0) / 30.0;  // Convert 30-second amount to ml/min
      WATER_FLOW_RATE = settings.flowRate;
      
      // Update water dispense duration
      WATER_DISPENSE_DURATION = (WATER_AMOUNT / WATER_FLOW_RATE) * 60 * 1000;
      
      lcd.clear();
      lcd.print("Flow rate set:");
      lcd.setCursor(0, 1);
      lcd.print(settings.flowRate);
      lcd.print(" ml/min");
      delay(2000);
      
      updateSettingsDisplay();
      return;
    }
    joystickButtonPrevious = joystickButtonPressed;
    delay(10);
  }
}

// Toggle calibration mode between Auto and Manual
void toggleCalibrationMode() {
  lcd.clear();
  lcd.print("Cal Mode: ");
  lcd.setCursor(0, 1);
  lcd.print(settings.useManualCalibration ? "MANUAL" : "AUTO");
  
  joystickButtonPrevious = true;
  delay(300);
  
  while (true) {
    readJoystick();
    unsigned long currentTime = millis();
    
    // Toggle mode on joystick movement
    if (currentTime - lastJoystickMove >= JOYSTICK_DEBOUNCE) {
      if (joystickXValue < 300 || joystickXValue > 700) {
        settings.useManualCalibration = !settings.useManualCalibration;
        USE_MANUAL_CALIBRATION = settings.useManualCalibration;
        
        lcd.clear();
        lcd.print("Cal Mode: ");
        lcd.setCursor(0, 1);
        lcd.print(settings.useManualCalibration ? ">> MANUAL <<" : ">> AUTO <<");
        
        lastJoystickMove = currentTime;
        digitalWrite(LED_PLASTIC, HIGH);
        delay(100);
        digitalWrite(LED_PLASTIC, LOW);
      }
    }
    
    // Exit on button press
    if (joystickButtonPressed && !joystickButtonPrevious) {
      lcd.clear();
      lcd.print("Mode set to:");
      lcd.setCursor(0, 1);
      lcd.print(settings.useManualCalibration ? "MANUAL" : "AUTO");
      
      if (settings.useManualCalibration) {
        calibration_factor = settings.manualCalibrationFactor;
        scale.set_scale(calibration_factor);
        calibration_complete = true;
      } else {
        // Check if auto calibration was previously done
        if (settings.autoCalibrationFactor != 0) {
          calibration_factor = settings.autoCalibrationFactor;
          scale.set_scale(calibration_factor);
          calibration_complete = true;
          
          lcd.clear();
          lcd.print("Using saved");
          lcd.setCursor(0, 1);
          lcd.print("auto cal factor");
          delay(1500);
        } else {
          calibration_complete = false;
          performAutoCalibration();
        }
      }
      
      delay(1000);
      updateSettingsDisplay();
      return;
    }
    joystickButtonPrevious = joystickButtonPressed;
    delay(10);
  }
}

// Display calibration point based on current mode
void displayCalibrationPoint() {
  if (settings.useManualCalibration) {
    // Manual mode - show adjustable manual calibration factor with live weight
    int tempValue = abs(settings.manualCalibrationFactor);
    if (tempValue < 1) tempValue = 1;
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Factor: ");
    lcd.print(tempValue);
    
    // Show current weight with current calibration
    scale.set_scale(-tempValue);
    delay(500); // Brief pause to show initial value
    
    joystickButtonPrevious = true;
    delay(300);
    
    while (true) {
      readJoystick();
      unsigned long currentTime = millis();
      
      // Update weight display every 200ms for live feedback
      static unsigned long lastWeightUpdate = 0;
      if (currentTime - lastWeightUpdate >= 200) {
        float weight = scale.get_units(3);
        if (weight < 0) weight = 0.0;
        
        lcd.setCursor(0, 0);
        lcd.print("Factor: ");
        lcd.print(tempValue);
        lcd.print("    ");
        
        lcd.setCursor(0, 1);
        lcd.print("Wt: ");
        lcd.print(weight, 2);
        lcd.print("g      ");
        
        lastWeightUpdate = currentTime;
      }
      
      if (currentTime - lastJoystickMove >= JOYSTICK_DEBOUNCE) {
        bool valueChanged = false;
        
        if (joystickXValue < 300) {  // Left - decrease
          if (tempValue > 1) {
            tempValue--;
            valueChanged = true;
          }
          lastJoystickMove = currentTime;
        }
        else if (joystickXValue > 700) {  // Right - increase
          tempValue++;
          valueChanged = true;
          lastJoystickMove = currentTime;
        }
        
        if (valueChanged) {
          scale.set_scale(-tempValue);
        }
      }
      
      if (joystickButtonPressed && !joystickButtonPrevious) {
        settings.manualCalibrationFactor = -tempValue;
        MANUAL_CALIBRATION_FACTOR = settings.manualCalibrationFactor;
        
        // Apply the new calibration factor
        calibration_factor = MANUAL_CALIBRATION_FACTOR;
        scale.set_scale(calibration_factor);
        
        updateSettingsDisplay();
        return;
      }
      joystickButtonPrevious = joystickButtonPressed;
      delay(10);
    }
  } else {
    // Auto mode - show view-only auto calibration factor
    lcd.clear();
    lcd.print("Auto Cal Point:");
    lcd.setCursor(0, 1);
    lcd.print("Factor: ");
    if (settings.autoCalibrationFactor == 0) {
      lcd.print("Not set");
    } else {
      lcd.print((int)settings.autoCalibrationFactor);
    }
    
    // Wait for button press to exit
    joystickButtonPrevious = true;
    delay(500);
    
    while (true) {
      readJoystick();
      if (joystickButtonPressed && !joystickButtonPrevious) {
        updateSettingsDisplay();
        return;
      }
      joystickButtonPrevious = joystickButtonPressed;
      delay(10);
    }
  }
}

// Test servo 1 (main gate)
void testServo1() {
  lcd.clear();
  lcd.print("Servo 1 Test");
  lcd.setCursor(0, 1);
  lcd.print("Move to control");
  
  joystickButtonPrevious = true;
  delay(500);
  
  while (true) {
    readJoystick();
    unsigned long currentTime = millis();
    
    if (currentTime - lastJoystickMove >= JOYSTICK_DEBOUNCE) {
      if (joystickXValue < 300) {  // Left - close
        servo1.write(SERVO1_CLOSED);
        lcd.setCursor(0, 1);
        lcd.print("CLOSED          ");
        lastJoystickMove = currentTime;
      }
      else if (joystickXValue > 700) {  // Right - open
        servo1.write(SERVO1_OPEN);
        lcd.setCursor(0, 1);
        lcd.print("OPEN            ");
        lastJoystickMove = currentTime;
      }
    }
    
    if (joystickButtonPressed && !joystickButtonPrevious) {
      servo1.write(SERVO1_CLOSED);
      updateSettingsDisplay();
      return;
    }
    joystickButtonPrevious = joystickButtonPressed;
    delay(10);
  }
}

// Test servo 3 (water dispenser guide)
void testServo3() {
  lcd.clear();
  lcd.print("Servo 3 Test");
  lcd.setCursor(0, 1);
  lcd.print("Move to control");
  
  int currentPosition = SERVO3_NEUTRAL;
  servo3.write(currentPosition);
  
  joystickButtonPrevious = true;
  delay(500);
  
  while (true) {
    readJoystick();
    unsigned long currentTime = millis();
    
    if (currentTime - lastJoystickMove >= JOYSTICK_DEBOUNCE) {
      if (joystickXValue < 300) {  // Left - plastic position
        currentPosition = SERVO3_PLASTIC;
        servo3.write(currentPosition);
        lcd.setCursor(0, 1);
        lcd.print("PLASTIC         ");
        lastJoystickMove = currentTime;
      }
      else if (joystickXValue > 700) {  // Right - metal position
        currentPosition = SERVO3_METAL;
        servo3.write(currentPosition);
        lcd.setCursor(0, 1);
        lcd.print("METAL           ");
        lastJoystickMove = currentTime;
      }
    }
    
    if (joystickButtonPressed && !joystickButtonPrevious) {
      servo3.write(SERVO3_NEUTRAL);
      updateSettingsDisplay();
      return;
    }
    joystickButtonPrevious = joystickButtonPressed;
    delay(10);
  }
}

// Handle water button with debounce
void handleWaterButton(unsigned long currentTime) {
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(WATER_BUTTON);
  
  // Detect button press
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    if (currentTime - lastButtonPress > BUTTON_DEBOUNCE_TIME) {
      if (waterDispensing) {
        // Stop dispensing if already running
        stopWaterDispense();
      } else {
        // Check credits
        if (credits >= WATER_CREDIT_COST) {
          credits -= WATER_CREDIT_COST;
          startWaterDispense(currentTime);
        } else {
          buzzWarning();
          // Flash error LED
          digitalWrite(LED_ERROR, HIGH);
          delay(500);
          digitalWrite(LED_ERROR, LOW);
        }
      }
      lastButtonPress = currentTime;
    }
  }
  
  lastButtonState = currentButtonState;
}

// Open main gate
void openGate1(unsigned long currentTime) {
  servo1.write(SERVO1_OPEN);
  gate1Open = true;
  gate1OpenTime = currentTime;
}

// Display bin level and allow threshold adjustment
void displayBinLevel(bool isPlastic) {
  lcd.clear();
  lcd.print(isPlastic ? "Plastic Bin" : "Metal Bin");
  
  int tempValue = isPlastic ? (settings.plasticFullAmount * 10) : (settings.metalFullAmount * 10);
  joystickButtonPrevious = true;
  delay(300);
  
  while (true) {
    readJoystick();
    unsigned long currentTime = millis();
    
    // Update display every 500ms to show live levels
    static unsigned long lastLevelUpdate = 0;
    if (currentTime - lastLevelUpdate >= 500) {
      float actualLevel = isPlastic ? plasticBinDistance : metalBinDistance;
      
      lcd.setCursor(0, 0);
      lcd.print(isPlastic ? "Plastic:" : "Metal:  ");
      lcd.print(actualLevel, 1);  // Show actual distance
      lcd.print("cm");
      
      lcd.setCursor(0, 1);
      lcd.print("Limit: ");
      lcd.print(tempValue / 10.0, 1);  // Show threshold limit
      lcd.print("cm    ");
      
      lastLevelUpdate = currentTime;
    }
    
    if (currentTime - lastJoystickMove >= JOYSTICK_DEBOUNCE) {
      if (joystickXValue < 300) {  // Left - decrease threshold
        if (tempValue > 10) tempValue--;  // Minimum 1.0cm
        lastJoystickMove = currentTime;
      }
      else if (joystickXValue > 700) {  // Right - increase threshold
        if (tempValue < 500) tempValue++;  // Maximum 50.0cm
        lastJoystickMove = currentTime;
      }
    }
    
    if (joystickButtonPressed && !joystickButtonPrevious) {
      // Save the adjusted threshold value
      if (isPlastic) {
        settings.plasticFullAmount = tempValue / 10.0;
        PLASTIC_FULL_AMOUNT = settings.plasticFullAmount;
      } else {
        settings.metalFullAmount = tempValue / 10.0;
        METAL_FULL_AMOUNT = settings.metalFullAmount;
      }
      
      lcd.clear();
      lcd.print("Threshold set:");
      lcd.setCursor(0, 1);
      lcd.print(tempValue / 10.0, 1);
      lcd.print("cm");
      delay(1500);
      
      updateSettingsDisplay();
      return;
    }
    joystickButtonPrevious = joystickButtonPressed;
    delay(10);
  }
}

// Manual water dispense function with input amount and interruption
void runManualWaterDispense() {
  lcd.clear();
  lcd.print("Dispense water");
  lcd.setCursor(0, 1);
  lcd.print("Amount: 0ml");
  
  // Input amount
  int dispenseAmount = 0;
  joystickButtonPrevious = true;
  delay(300);
  
  while (true) {
    readJoystick();
    unsigned long currentTime = millis();
    
    if (currentTime - lastJoystickMove >= JOYSTICK_DEBOUNCE) {
      if (joystickXValue < 300) {  // Left - decrease amount
        if (dispenseAmount > 0) dispenseAmount -= 1;  // 1ml increments
        lastJoystickMove = currentTime;
        lcd.setCursor(0, 1);
        lcd.print("Amount: ");
        lcd.print(dispenseAmount);
        lcd.print("ml    ");
      }
      else if (joystickXValue > 700) {  // Right - increase amount
        if (dispenseAmount < 2000) dispenseAmount += 1;  // 1ml increments, max 2000ml
        lastJoystickMove = currentTime;
        lcd.setCursor(0, 1);
        lcd.print("Amount: ");
        lcd.print(dispenseAmount);
        lcd.print("ml    ");
      }
    }
    
    if (joystickButtonPressed && !joystickButtonPrevious) {
      break;
    }
    joystickButtonPrevious = joystickButtonPressed;
    delay(10);
  }
  
  // Calculate dispense duration
  unsigned long dispenseDuration = (dispenseAmount / WATER_FLOW_RATE) * 60 * 1000;
  
  lcd.clear();
  lcd.print("Dispensing...");
  lcd.setCursor(0, 1);
  lcd.print(dispenseAmount);
  lcd.print("ml");
  
  // Start pump
  digitalWrite(RELAY_PUMP, LOW);  // Turn ON pump
  buzzShort();
  
  unsigned long dispenseStartTime = millis();
  bool interrupted = false;
  
  // Dispense with interruption check
  while (millis() - dispenseStartTime < dispenseDuration) {
    // Check water button
    if (digitalRead(WATER_BUTTON) == LOW) {
      interrupted = true;
      break;
    }
    
    // Check joystick button
    readJoystick();
    if (joystickButtonPressed) {
      interrupted = true;
      break;
    }
    
    delay(10);
  }
  
  // Stop pump
  digitalWrite(RELAY_PUMP, HIGH); // Turn OFF pump
  buzzShort();
  
  lcd.clear();
  if (interrupted) {
    lcd.print("Dispense stopped");
    lcd.setCursor(0, 1);
    lcd.print("by user");
  } else {
    lcd.print("Dispense");
    lcd.setCursor(0, 1);
    lcd.print("Complete!");
  }
  delay(2000);
  
  updateSettingsDisplay();
}
