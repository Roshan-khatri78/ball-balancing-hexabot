🚀 ASTRA - AI Ball Balancing Platform
<div align="center">
Show Image
Show Image
Show Image
Intelligent hexagonal platform that balances a ball using AI and computer vision
Final Year Project | IOE 2078 Batch | Purwanchal Campus
Features • Hardware • Quick Start • Team
</div>

🎯 What It Does

Tracks ball position using computer vision
AI-powered adaptive control system
Real-time learning and correction
Interactive display interface


✨ Features
🧠 Smart Control - AI learns and adapts in real-time
📹 Vision Tracking - OpenCV ball detection
🎮 Touch Interface - 2" TFT display + rotary encoder
⚡ Dual ESP32 - Separate motor & UI control
📊 Live Graphs - Real-time performance monitoring

🔧 Hardware
Current Build (Phase 2)
💰 Cost: ~$90
🎯 Accuracy: ±0.05°
⚙️ Motors: NEMA 17 Steppers
PartQuantityESP32 Dev Board2NEMA 17 Stepper Motor3TB6600 Stepper Driver324V 5A SMPS12.0" TFT LCD1Rotary Encoder1USB Webcam (720p)1
Budget Build (Phase 1)
💰 Cost: ~$35
🎯 Accuracy: ±1-2°
⚙️ Motors: MG996R Servos
PartQuantityESP32 Dev Board1MG996R Servo Motor316x2 I2C LCD1Push Buttons5USB Webcam (720p)15V Power Supply1

📊 Performance
MetricValueControl Loop50 HzPosition Accuracy±0.005 unitsStabilization5-30 secondsAI Learning Time~4 seconds

🚀 Quick Start
Phase 1 - Simulation & Servo Build
bashgit clone https://github.com/Roshan-khatri78/ball-balancing-hexabot.git
cd ball-balancing-hexabot/Phase1
# Follow Phase1/README.md
Phase 2 - Stepper Build (Coming Soon)
bashcd Phase2
# Follow Phase2/README.md

💡 Which Build?
Choose Servos if you want:

Low cost ($35)
Simple wiring
Learning project

Choose Steppers if you need:

High precision (±0.05°)
Research quality
Publication-ready results


👥 Team
IOE 2078 Batch - Purwanchal Campus, Dharan
MemberRoleSneha YadavAI/MLRudra KhatriHardwareBishakha PokhrelFirmwareSusant DahalTesting
Supervisor: Bishnu Chaudhary

📂 Structure
ball-balancing-hexabot/
├── Phase1/              # Simulation + Servo prototype
│   ├── simulation/
│   ├── docs/
│   └── ai_models/
│
├── Phase2/              # Stepper implementation
│   ├── firmware/
│   │   ├── motor_controller/
│   │   └── ui_controller/
│   ├── hardware/
│   └── visualization/
│
└── shared/              # Common resources

🤝 Contributing
Contributions welcome!
bashfork → branch → commit → push → pull request

📜 License
MIT License © 2024 ASTRA Team

📞 Contact
📧 rudrakhatri456@gmail.com
🐛 Report Issues
💬 Discussions

<div align="center">
⭐ Star this repo if you find it useful!
Made with ❤️ by IOE 2078 Batch Students
</div>
