# User Manual - Ball Balancing Hexabot

## Quick Start Guide

### System Requirements
- **Hardware**: ESP32, 3x MG996R servos, LCD display, webcam
- **Software**: Python 3.8+, Arduino IDE
- **Operating System**: Windows 10+, Linux Ubuntu 18+, or macOS 10.14+

### First Time Setup

#### 1. Hardware Assembly
1. Connect servos to ESP32 (pins 18, 19, 21)
2. Connect LCD display via I2C (SDA: pin 22, SCL: pin 23)
3. Connect 5 buttons for menu navigation
4. Position webcam above platform
5. Power system with 5V, 3A supply

#### 2. Software Installation
```bash
# Clone repository
git clone https://github.com/Roshan-khatri78/ball-balancing-hexabot.git
cd ball-balancing-hexabot

# Install Python dependencies
pip install -r requirements.txt

# Upload Arduino code
# Open firmware/hexabot_main.ino in Arduino IDE
# Select ESP32 board, choose correct port, and upload
```

#### 3. Initial Configuration
1. Edit `software/config.py` with your settings:
   - Camera index (usually 0)
   - Serial port (COM3 on Windows, /dev/ttyUSB0 on Linux)
   - Ball color for detection

## Operating the System

### Main Interface (LCD Menu)

The system uses a 5-button interface displayed on the LCD:

```
┌─────────────────┐
│ Ball Balancing  │
│ > Start System  │
└─────────────────┘
```

**Button Functions:**
- **Up/Down**: Navigate menu items
- **Left/Right**: Adjust values (when applicable)
- **Select**: Confirm selection
