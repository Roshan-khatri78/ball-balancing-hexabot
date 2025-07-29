# 🟠 Ball Balancing Hexabot

A final-year BEI major project (2078 batch) from **Purwanchal Campus, Dharan-08, Sunsari** under **IOE**, this system balances a ball on a hexagonal platform using real-time image processing and servo control.

---

## 🎯 Project Overview

This project uses:
- **OpenCV** for real-time ball tracking via webcam
- **PID control** to calculate corrections
- **ESP32** to drive 3 servo motors (MG996R)
- A **hexagonal tilting base** to stabilize the ball dynamically

The system simulates a feedback-controlled stabilizer and can be visualized or tuned through a UI or display.

---

## 🎮 Interactive LCD & Button Control

The system includes a **5-button interface** with an **I2C LCD display** allowing users to interactively select commands such as:

- **Start Balancing**
- **Move Left / Right / Forward / Backward**
- **Center Ball**
- **PID Mode: Manual / Auto**
- **Exit / Reset**

### How it works:
- The **LCD** continuously shows the current mode or command.
- Users navigate through the menu with:
  - 4 directional push buttons (Up, Down, Left, Right)
  - 1 Select button (OK)
- The selected menu item is highlighted on the LCD.
- Pressing OK executes the corresponding command (e.g., servo angle adjustment).
- Advanced features may include real-time PID tuning, live ball position display, and status updates.

---

## 🧠 Features

- Ball tracking using Python + OpenCV
- PID controller in Python
- Servo angle control via ESP32
- Interactive LCD menu with 5-button navigation
- Optional Flask UI or I2C LCD display
- Modular and extendable design
- Simulation + real hardware implementation

---

## 🏗️ Project Structure

- `ball-balancing-hexabot/`
  - `docs/`             - Diagrams and documentation
  - `hardware/`         - CAD models for platform & mounts
  - `firmware/`         - ESP32/Arduino code for servos, LCD, buttons
  - `software/`         - Python OpenCV & PID logic
  - `web_interface/`    - (Optional) Live tuning interface
  - `simulation/`       - (Optional) Simulated environment
  - `requirements.txt`  - Python dependencies
  - `README.md`         - This file
  - `.gitignore`        - Git ignore rules

---

## 👨‍💻 Technologies Used

- Python 3
- OpenCV
- ESP32 + Arduino IDE
- MG996R Servo Motors
- PID Control Logic
- I2C LCD Display
- 5 Push Buttons for User Input
- Flask (optional UI)

---

## 👥 Team Members — IOE 2078 Batch

- Rudra Khatri  
- Sneha Yadav  
- Bishakha Pokhrel  
- Sushant Dahal  

---

## 📜 License

This project is licensed under the MIT License. See the `LICENSE` file for details.
