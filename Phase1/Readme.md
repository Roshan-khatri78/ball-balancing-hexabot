# 🚀 Project ASTRA - Phase 1: AI-Enhanced Ball Balancing Simulation

[![MIT License](https://img.shields.io/badge/License-MIT-green.svg)](../LICENSE)
[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://python.org)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.0+-red.svg)](https://opencv.org)
[![ESP32](https://img.shields.io/badge/ESP32-Arduino-orange.svg)](https://www.espressif.com/en/products/socs/esp32)

---

## 🎯 Phase 1 Overview

This phase focuses on **simulation and AI algorithm development** for the ASTRA ball balancing hexabot project.

- Realistic ball physics and platform simulation using Python.
- AI-enhanced PID control algorithms validated in simulation.
- ESP32 firmware adapted to interface with simulation environment.

The **physical hardware integration and deployment will be done in Phase 2** and uploaded separately.

---

## 🧪 Simulation Components

### Python Simulation
- Ball and platform physics modeled with OpenCV visualization.
- AI PID gain optimization, predictive error compensation, and disturbance rejection.
- Real-time data plotting with 8-panel dashboard.

### Firmware (ESP32)
- Arduino code tested against simulated inputs.
- AI-enhanced PID control loops.
- Serial communication for data streaming to Python visualizer.

---

## 🔧 Installation & Setup

### Clone repository and enter Phase1 folder:
```bash
git clone https://github.com/Roshan-khatri78/ball-balancing-hexabot.git
cd ball-balancing-hexabot/Phase1

Python environment setup:

python -m venv astra_env
# Activate:
# Windows: astra_env\Scripts\activate
# Linux/macOS: source astra_env/bin/activate

pip install -r requirements.txt

Running simulation visualization:

python Astra_plot.py

📖 Usage Guide
Simulation Controls

    Adjust PID parameters and AI modes in config.yaml.

    Use Python scripts to simulate various trajectory shapes.

    Visualize ball trajectory, servo angles, AI performance in real-time.

Firmware Testing

    Upload midtermdefense.ino to ESP32 for firmware testing.

    Use serial communication to connect ESP32 with simulation scripts.

🧠 AI Algorithm Details

    Adaptive PID tuning with 5 gain sets.

    Predictive compensation using historical error trends.

    Rolling average-based disturbance rejection.

    Trajectory pattern recognition for AI adaptation.

🏗️ System Architecture (Simulation Focus)

graph TB
    ESP32 -- Simulated control signals --> Servos
    Python -- Ball physics & vision --> Simulation display
    ESP32 <-- Serial data --> Python visualizer

📂 Folder Structure

Phase1/
├── firmware/
│   └── midtermdefense.ino
├── simulation/
│   ├── Astra_plot.py
│   ├── physics_simulator.py
│   ├── ai_training_sim.py
│   └── performance_benchmarks.py
├── docs/
│   ├── major_proposal_optimized.pdf
│   ├── ai_algorithms_explained.md
│   ├── performance_analysis.md
│   └── user_manual.md
├── requirements.txt
└── config.yaml

🔮 Future Work: Phase 2

    Hardware integration with servos, ESP32, LCD, buttons.

    Physical platform construction and testing.

    Real-time hardware AI deployment.

    Documentation and demos of physical system.

🤝 Contributing & Support

    Fork, clone, and submit PRs for simulation improvements.

    Report issues and ask questions via GitHub Issues/Discussions.

    Contact: rudrakhatri456@gmail.com

📜 License

This project is licensed under the MIT License. See LICENSE.
<div align="center">
🌟 Phase 1 Simulation — Building the foundation for intelligent control 🌟

Physical implementation coming soon in Phase 2!

Made with ❤️ by IOE 2078 Batch Students
</div> ```
