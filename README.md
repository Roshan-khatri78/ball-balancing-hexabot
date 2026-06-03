<div align="center">

# ⚡ ASTRA — AI Ball Balancing Platform

**An intelligent hexagonal platform that balances a ball using AI and computer vision**

![MIT License](https://img.shields.io/badge/License-MIT-00e5a0?style=flat-square)
![ESP32](https://img.shields.io/badge/MCU-ESP32-blue?style=flat-square)
![OpenCV](https://img.shields.io/badge/Vision-OpenCV-red?style=flat-square)
![C++](https://img.shields.io/badge/Language-C%2B%2B-orange?style=flat-square)
![IOE 2078](https://img.shields.io/badge/IOE-2078%20Batch-purple?style=flat-square)

*Final Year Project · Purwanchal Campus*

</div>

---

## 🧠 What It Does

| Feature | Description |
|---|---|
| 👁 **Vision Tracking** | OpenCV ball detection via USB webcam at 720p/15fps |
| 🤖 **Smart Control** | AI learns and adapts in real-time — no manual tuning |
| ⚡ **Dual ESP32** | Separate motor and UI control for responsive operation |
| 📊 **Live Graphs** | 2" TFT display with rotary encoder for real-time monitoring |

---

## 📊 Performance

| Metric | Value |
|---|---|
| Control Loop | 50 Hz |
| Position Accuracy | ±0.005 units |
| Stabilization Time | ~5 seconds |
| AI Learning Time | ~4 seconds |

---

## 🔧 Choose Your Build

### Phase 1 — Servo Build `~$35`

> Best for: low cost, simple wiring, learning projects

| Component | Spec |
|---|---|
| Motors | MG996R Servos ×3 |
| Board | ESP32 Dev Board |
| UI | I2C LCD + Push Buttons |
| Camera | USB Webcam 720p |
| Power | 15V PSU |
| Accuracy | ±1–2° |

### Phase 2 — Stepper Build `~$90` ⭐ Recommended

> Best for: high precision, research, publication-ready results

| Component | Spec |
|---|---|
| Motors | NEMA 17 Steppers ×3 |
| Driver | TB6600 ×3 @ 24V 5A |
| Board | ESP32 Dev Board |
| UI | 2" TFT + Rotary Encoder |
| Camera | USB Webcam 720p |
| Accuracy | ±0.05° |

---

## 🚀 Quick Start

**1. Clone the repo**
```bash
git clone https://github.com/Roshan-khatri78/ball-balancing-hexabot.git
cd ball-balancing-hexabot
```

**2. Phase 1 — Simulation & Servo Build**
```bash
cd Phase1
# Follow Phase1/README.md
```

**3. Phase 2 — Stepper Build**
```bash
cd Phase2
# Follow Phase2/README.md
```

---

## 📁 Project Structure

```
ball-balancing-hexabot/
├── Phase1/               # Simulation & Servo prototype
│   ├── simulation/
│   ├── ai_models/
│   └── firmware/
├── Phase2/               # Stepper implementation
│   ├── firmware/
│   ├── motor_controller/
│   └── ui_controller/
├── docs/
│   └── hardware/
├── visualization/
├── shared/               # Common resources
└── README.md
```

---

## 👥 Team · IOE 2078 · Purwanchal Campus

| Name | Role |
|---|---|
| Dharan Member | ML / AI |
| Sneha Yadav | Firmware |
| Rudra Khatri | Hardware |
| Bishaka Pokhrel | Firmware |
| Susant Daha | Testing |
| Bishnu Chaudhary | Supervisor |

---

## 🤝 Contributing

Contributions welcome!

```bash
git checkout -b your-feature
git commit -m "your message"
git push origin your-feature
# open a pull request
```

---

<div align="center">

MIT License © 2024 ASTRA Team

📧 [rudrakhatri456@gmail.com](mailto:rudrakhatri456@gmail.com) · ⭐ Star this repo if you find it useful!

*Made with ❤️ by IOE 2078 Batch Students*

</div>
