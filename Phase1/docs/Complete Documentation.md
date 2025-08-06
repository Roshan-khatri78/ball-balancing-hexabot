# Ball Balancing Hexabot - Complete Documentation

## Table of Contents
1. [Project Overview](#project-overview)
2. [Quick Start Guide](#quick-start-guide)
3. [Hardware Setup](#hardware-setup)
4. [Software Installation](#software-installation)
5. [Code Structure](#code-structure)
6. [API Documentation](#api-documentation)
7. [Usage Examples](#usage-examples)
8. [Troubleshooting](#troubleshooting)
9. [Contributing](#contributing)
10. [Appendix](#appendix)

---

## Project Overview

### Description
A final-year BEI major project (2078 batch) from Purwanchal Campus, Dharan-08, Sunsari under IOE. This system balances a ball on a hexagonal platform using real-time image processing and servo control.

### Key Features
- **Real-time Ball Tracking**: OpenCV-based computer vision for ball detection
- **PID Control System**: Automatic feedback control for platform stabilization
- **Interactive Interface**: 5-button menu system with I2C LCD display
- **Servo Control**: 3x MG996R servo motors for hexagonal platform tilting
- **Simulation Environment**: Matplotlib-based physics simulation for testing
- **Modular Design**: Separate firmware, software, and simulation modules

### Technical Specifications
- **Microcontroller**: ESP32 Development Board
- **Servo Motors**: 3x MG996R High Torque Servo Motors
- **Display**: 16x2 I2C LCD Display
- **Input**: 5 Push Buttons (Up, Down, Left, Right, Select)
- **Camera**: USB Webcam for ball tracking
- **Platform**: Custom hexagonal tilting base
- **Communication**: Serial communication between Python and ESP32

---

## Quick Start Guide

### Prerequisites
- Python 3.8+
- Arduino IDE
- USB Webcam
- ESP32 Development Board
- Required hardware components (see [Hardware Setup](#hardware-setup))

### Installation Steps

1. **Clone the Repository**
   ```bash
   git clone https://github.com/Roshan-khatri78/ball-balancing-hexabot.git
   cd ball-balancing-hexabot
   ```

2. **Install Python Dependencies**
   ```bash
   pip install -r requirements.txt
   ```

3. **Upload Arduino Firmware**
   ```bash
   # Open firmware/hexabot_main.ino in Arduino IDE
   # Select ESP32 board and appropriate port
   # Upload the code
   ```

4. **Run Simulation (Optional)**
   ```bash
   python simulation/ball_simulation.py
   ```

5. **Run Main Application**
   ```bash
   python software/main_controller.py
   ```

---

## Hardware Setup

### Components List

| Component | Quantity | Specifications | Purpose |
|-----------|----------|----------------|---------|
| ESP32 Development Board | 1 | 32-bit dual-core processor | Main controller |
| MG996R Servo Motors | 3 | High torque, metal gear | Platform tilting |
| 16x2 LCD Display | 1 | I2C interface | User interface |
| Push Buttons | 5 | Momentary contact | Menu navigation |
| USB Webcam | 1 | 720p minimum | Ball tracking |
| Ball | 1 | Lightweight, solid color | Balancing object |
| Platform Material | 1 | Hexagonal base | Ball balancing surface |
| Connecting Wires | Multiple | Jumper wires | Connections |
| Power Supply | 1 | 5V, 3A minimum | Servo power |

### Wiring Diagram

```
ESP32 Pin Connections:
├── Servo 1 (X-axis): GPIO 21
├── Servo 2 (Y-axis): GPIO 22  
├── Servo 3 (Z-axis): GPIO 23
├── LCD SDA: GPIO 04
├── LCD SCL: GPIO 15
├── Button cancel: GPIO 13 
├── Button stop: GPIO 27
├── Button Left: GPIO 14
├── Button Right: GPIO 33
└── Button Select: GPIO 26
```

### Assembly Instructions

1. **Platform Construction**
   - Create hexagonal platform (30cm diameter recommended)
   - Mount servo motors at 120° intervals
   - Ensure smooth tilting mechanism

2. **Electronic Connections**
   - Connect servos to ESP32 PWM pins
   - Wire LCD display via I2C
   - Connect buttons with pull-up resistors
   - Provide adequate power supply for servos

3. **Camera Setup**
   - Position webcam above platform
   - Ensure clear view of entire platform
   - Good lighting conditions recommended

---

## Software Installation

### Environment Setup

1. **Python Virtual Environment**
   ```bash
   # Create virtual environment
   python -m venv hexabot_env
   
   # Activate (Windows)
   hexabot_env\Scripts\activate
   
   # Activate (Linux/Mac)
   source hexabot_env/bin/activate
   ```

2. **Install Dependencies**
   ```bash
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

### Arduino IDE Configuration

1. **Install ESP32 Board Package**
   - File → Preferences → Additional Board Manager URLs
   - Add: `https://dl.espressif.com/dl/package_esp32_index.json`
   - Tools → Board → Boards Manager → Search "ESP32" → Install

2. **Required Libraries**
   ```cpp
   // Install via Library Manager:
   - Servo.h (built-in)
   - LiquidCrystal_I2C
   - Wire.h (built-in)
   ```

---

## Code Structure

### Directory Organization

```
ball-balancing-hexabot/
├── firmware/                    # Arduino/ESP32 Code
│   ├── hexabot_main.ino        # Main Arduino sketch
│   ├── servo_control.h         # Servo motor control
│   ├── lcd_interface.h         # LCD and button handling
│   ├── serial_comm.h           # Serial communication
│   └── config.h                # Pin definitions & constants
├── software/                    # Python Application
│   ├── main_controller.py      # Main application entry
│   ├── ball_tracker.py         # OpenCV ball detection
│   ├── pid_controller.py       # PID control implementation
│   ├── serial_handler.py       # ESP32 communication
│   ├── calibration.py          # System calibration
│   └── config.py               # Configuration settings
├── simulation/                  # Simulation Environment
│   ├── ball_simulation.py      # Main simulation (Astra_plot)
│   ├── physics_engine.py       # Ball physics simulation
│   ├── platform_dynamics.py    # Platform movement simulation
│   ├── pid_simulation.py       # PID controller testing
│   └── visualization.py        # Matplotlib visualization
├── hardware/                   # Hardware Documentation
│   ├── schematics/             # Circuit diagrams
│   ├── cad_models/             # 3D models
│   └── assembly_guide.md       # Assembly instructions
├── docs/                       # Project Documentation
│   ├── major_proposal_optimized.pdf
│   ├── api_reference.md        # API documentation
│   ├── user_manual.md          # User guide
│   └── troubleshooting.md      # Common issues
└── web_interface/              # Optional Web Interface
    ├── app.py                  # Flask application
    ├── templates/              # HTML templates
    └── static/                 # CSS/JS files
```

### Key Modules Description

#### Firmware (Arduino/ESP32)
- **hexabot_main.ino**: Main control loop, menu system, servo coordination
- **servo_control.h**: PWM signal generation, angle calculations
- **lcd_interface.h**: Menu display, button input handling
- **serial_comm.h**: Communication protocol with Python application

#### Software (Python)
- **main_controller.py**: Application entry point, main control loop
- **ball_tracker.py**: Computer vision, ball detection and tracking
- **pid_controller.py**: PID algorithm implementation and tuning
- **serial_handler.py**: Serial communication, command parsing

#### Simulation
- **ball_simulation.py**: Complete simulation environment with visualization
- **physics_engine.py**: Ball movement physics, platform dynamics
- **pid_simulation
