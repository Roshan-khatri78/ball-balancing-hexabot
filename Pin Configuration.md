# Pin Configuration - Ball Balancing Hexabot

## ESP32 Pin Assignments

### Servo Motors
```cpp
#define SERVO_1_PIN    21  // D21 - First servo motor
#define SERVO_2_PIN    22  // D22 - Second servo motor  
#define SERVO_3_PIN    23  // D23 - Third servo motor
```

### LCD Display (I2C)
```cpp
#define LCD_SDA_PIN    4   // D4 - I2C Data line
#define LCD_SCL_PIN    15  // D15 - I2C Clock line
```

### Push Buttons
```cpp
#define BUTTON_LEFT    2   // D2 - Left navigation
#define BUTTON_SELECT  26  // D26 - Select/OK button
#define BUTTON_RIGHT   33  // D33 - Right navigation  
#define BUTTON_BACK    13  // D13 - Back/Cancel button
#define BUTTON_STOP    27  // D27 - Emergency stop
```

## Power System Configuration

### Input Power
- **Input Voltage**: 20V DC
- **Step-down Output**: 5V DC
- **Current Capacity**: 3A minimum
- **Ground**: Common ground shared across all components

### Power Distribution
```
20V DC Input
     │
Step-down Converter (20V → 5V, 3A)
     │
     ├── ESP32 Development Board (5V, GND)
     ├── Servo 1 (5V, GND, PWM Signal: D21)
     ├── Servo 2 (5V, GND, PWM Signal: D22)
     ├── Servo 3 (5V, GND, PWM Signal: D23)  
     ├── LCD Display (5V, GND, SDA: D4, SCL: D15)
     └── Push Buttons (GND, Pull-up signals to: D2, D26, D33, D13, D27)
```

## Circuit Diagram

```
ESP32 Development Board
┌─────────────────────────┐
│ D21 ──────── Servo 1    │
│ D22 ──────── Servo 2    │
│ D23 ──────── Servo 3    │
│                         │
│ D4 (SDA) ─── LCD SDA    │
│ D15 (SCL) ── LCD SCL    │
│                         │
│ D2 ─────── Left Button  │
│ D26 ────── Select Button│
│ D33 ────── Right Button │
│ D13 ────── Back Button  │
│ D27 ────── Stop Button  │
│                         │
│ 5V ────────── Power In  │
│ GND ───────── Ground    │
└─────────────────────────┘
         │
    Common Ground
         │
Step-down Converter
20V DC ──► 5V DC (3A)
```

## Arduino Code Pin Setup

```cpp
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Pin definitions
#define SERVO_1_PIN    21  // D21
#define SERVO_2_PIN    22  // D22
#define SERVO_3_PIN    23  // D23

#define LCD_SDA_PIN    4   // D4
#define LCD_SCL_PIN    15  // D15

#define BUTTON_LEFT    2   // D2
#define BUTTON_SELECT  26  // D26
#define BUTTON_RIGHT   33  // D33
#define BUTTON_BACK    13  // D13
#define BUTTON_STOP    27  // D27

// Component initialization
Servo servo1, servo2, servo3;
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
    // Initialize servos
    servo1.attach(SERVO_1_PIN);
    servo2.attach(SERVO_2_PIN);
    servo3.attach(SERVO_3_PIN);
    
    // Initialize I2C for LCD
    Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
    lcd.init();
    lcd.backlight();
    
    // Initialize buttons with pull-up
    pinMode(BUTTON_LEFT, INPUT_PULLUP);
    pinMode(BUTTON_SELECT, INPUT_PULLUP);
    pinMode(BUTTON_RIGHT, INPUT_PULLUP);
    pinMode(BUTTON_BACK, INPUT_PULLUP);
    pinMode(BUTTON_STOP, INPUT_PULLUP);
    
    // Initialize serial communication
    Serial.begin(115200);
}
```

## Wiring Notes

### Servo Connections
- Each servo requires: 5V (red), GND (black), Signal (yellow/white)
- Signal wires connect to D21, D22, D23 respectively
- Power and ground from step-down converter

### LCD Connections  
- VCC → 5V from step-down converter
- GND → Common ground
- SDA → D4 (ESP32 GPIO 4)
- SCL → D15 (ESP32 GPIO 15)

### Button Connections
- One side of each button → respective GPIO pin
- Other side of each button → GND
- Internal pull-up resistors enabled in software
- No external resistors needed

### Power Supply Requirements
- **Minimum**: 3A at 5V for 3 servos + ESP32 + LCD
- **Recommended**: 5A at 5V for reliable operation
- Step-down converter must handle full load current
- Add large capacitors (1000μF) near servos to reduce noise

## Testing Pin Configuration

### Individual Component Tests

```cpp
// Test servo movement
void testServos() {
    servo1.write(90);  // Center position
    servo2.write(90);
    servo3.write(90);
    delay(1000);
    
    servo1.write(45);   // Test movement
    delay(500);
    servo1.write(135);
    delay(500);
    servo1.write(90);   // Return to center
}

// Test LCD display
void testLCD() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Pin Test: LCD");
    lcd.setCursor(0, 1);
    lcd.print("SDA:D4 SCL:D15");
    delay(2000);
}

// Test button inputs
void testButtons() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Press buttons:");
    
    while(true) {
        if (!digitalRead(BUTTON_LEFT)) {
            lcd.setCursor(0, 1);
            lcd.print("LEFT (D2)      ");
        }
        else if (!digitalRead(BUTTON_SELECT)) {
            lcd.setCursor(0, 1);
            lcd.print("SELECT (D26)   ");
        }
        else if (!digitalRead(BUTTON_RIGHT)) {
            lcd.setCursor(0, 1);
            lcd.print("RIGHT (D33)    ");
        }
        else if (!digitalRead(BUTTON_BACK)) {
            lcd.setCursor(0, 1);
            lcd.print("BACK (D13)     ");
        }
        else if (!digitalRead(BUTTON_STOP)) {
            lcd.setCursor(0, 1);
            lcd.print("STOP (D27)     ");
        }
        delay(100);
    }
}
```

## Troubleshooting Pin Issues

### Common Problems

**Servo not moving:**
- Check 5V power supply capacity
- Verify signal wire connected to correct pin (D21/D22/D23)
- Test with servo.write() commands
- Ensure common ground connection

**LCD not displaying:**
- Verify I2C address (0x27 or 0x3F)
- Check SDA to D4, SCL to D15 connections
- Try I2C scanner to detect LCD
- Confirm 5V power to LCD

**Buttons not responding:**
- Verify INPUT_PULLUP enabled in code
- Check button wiring to correct pins
- Test with digitalRead() and Serial output
- Confirm ground connections

**Power issues:**
- Measure actual voltage at components (should be 5V)
- Check current draw under load
- Add capacitors if voltage drops during servo movement
- Verify step-down converter capacity

## Pin Change Procedure

If you need to modify pin assignments:

1. **Update firmware code:**
   ```cpp
   #define NEW_PIN_NAME  new_pin_number
   ```

2. **Update this documentation**

3. **Test new configuration thoroughly**

4. **Update circuit diagrams**

5. **Document reasons for change**

---

*Pin configuration verified for ESP32 with 20V→5V power system*
