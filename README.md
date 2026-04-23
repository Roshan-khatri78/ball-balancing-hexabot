🚀 Project ASTRA - AI-Enhanced Ball Balancing Hexabot
Show Image
Show Image
Show Image
Show Image
ASTRA (Autonomous Stabilization Through Reinforced Adaptation) - A cutting-edge AI-enhanced ball balancing system that combines computer vision, machine learning, and adaptive control theory.

A final-year BEI major project (2078 batch) from Purwanchal Campus, Dharan-08, Sunsari under IOE.


🎯 Project Overview
ASTRA represents the next generation of control systems, integrating classical control theory with modern artificial intelligence to create an autonomous ball balancing platform that learns and adapts in real-time.
This intelligent system achieves superior ball stabilization through AI-powered PID optimization, predictive error compensation, and real-time learning algorithms.

🧠 Core AI Enhancements

Adaptive PID Gain Tuning: Real-time parameter optimization
Predictive Error Compensation: Machine learning-based future error prediction
Disturbance Rejection: Intelligent detection and compensation for systematic errors
Pattern Recognition: Adaptive response to different movement patterns
Performance Optimization: Continuous learning and improvement


🚀 Key System Features

✨ Multi-mode Operation: Static positioning, dynamic shape tracing, AI learning modes
📊 Real-time Visualization: Advanced 8-panel monitoring dashboard
🎮 Interactive Interface: 2.0" TFT LCD with intuitive rotary encoder navigation
🔄 Dual ESP32 Architecture: Dedicated controllers for motor control and user interface
⚙️ Precision Stepper Control: NEMA 17 motors with TB6600 drivers for accurate positioning


📋 Development Phases
This project is structured in two distinct phases:
📊 Phase 1: Simulation & Algorithm Development
Status: ✅ Complete | Location: Phase1/ directory
Phase 1 focuses on algorithm development, simulation, and initial hardware prototyping:

AI algorithm design and testing
Control system simulation
Performance analysis and optimization
Comprehensive documentation and research
Initial Hardware: MG996R Servo Motors (cost-effective prototype)

📁 View Phase 1 Details

🤖 Phase 2: Hardware Implementation & Integration
Status: 🚧 In Development | Location: Phase2/ directory (Coming Soon)
Phase 2 implements the complete physical system with upgraded stepper-based precision control:

Dual ESP32 firmware development (motor control + UI/encoder)
TB6600 stepper driver integration
TFT display interface implementation
Hardware assembly and integration
Real-time control system implementation
Performance validation and testing


🔄 Hardware Evolution: Servos → Steppers
After successful algorithm validation in Phase 1 using servo motors, we upgraded to stepper motors in Phase 2 for enhanced performance and precision.
Why We Changed from Servos to Steppers
FeatureServo Motors (Phase 1)Stepper Motors (Phase 2)ImprovementPosition Accuracy±1-2° (analog feedback)±0.05° (1/32 micro-stepping)20x betterPosition HoldingDrift under loadZero drift, rock-solidPerfect stabilityControl ResponseNon-linear, speed-dependentDeterministic, predictableEasier PID tuningResolution~0.5° minimum movement0.028° per micro-step18x finerTotal Steps/Rev~360 positions6400 micro-steps17x more precise
Results:

✅ 20x improvement in position repeatability
✅ Better ball stability and reduced jitter
✅ Simplified PID tuning and better AI prediction
✅ Smoother platform adjustments for fine ball control


⚖️ Cost vs Performance Comparison
AspectPhase 1: Servo-BasedPhase 2: Stepper-Based💰 Total Cost~$30-40~$80-100🎯 Position Accuracy±1-2°±0.05°🔌 ComplexitySimple (3 wires/motor)Moderate (driver required)⚡ Power SupplyBattery OKSMPS recommended📈 PerformanceGood for learningExcellent for precision🎓 Best ForBudget builds, educationResearch, high performance

💡 Which Should You Build?
Choose Servos (Phase 1) if:

✅ You're on a tight budget ($30-40 total)
✅ This is your first ball balancing project
✅ You want quick prototyping and simple wiring
✅ Educational demonstration is the primary goal
✅ You're okay with ±1-2° accuracy

Choose Steppers (Phase 2) if:

✅ You want maximum performance and precision
✅ You're documenting research or academic work
✅ You need repeatable, publishable results
✅ Budget allows for ~$80-100 investment
✅ You want professional-grade control system


💚 Both approaches are valid! Phase 1 proved the AI algorithms work excellently with servos. Phase 2 pushes the performance envelope with steppers for our academic research goals.


🔧 Hardware Requirements
Phase 2: Current Implementation (Stepper-Based - High Precision)
ComponentSpecificationQtyPurposeESP32 Dev Board240MHz Dual-core, WiFi+BT2Motor control & UI/EncoderNEMA 17 Stepper1.8° step angle, 40-60 N·cm3Platform actuationTB6600 Driver4A, 9-42VDC3Motor controlSMPS Power Supply24V, 5A minimum1System power2.0" TFT LCDSPI interface, 240x3201User interfaceRotary EncoderWith push button1Navigation controlUSB Webcam720p minimum1Computer visionHexagonal PlatformLightweight, balanced1Ball surface
Estimated Cost: ~$80-100

Phase 1: Alternative Implementation (Servo-Based - Budget-Friendly)
ComponentSpecificationQtyPurposeCostESP32 Dev Board240MHz Dual-core1Main controller~$8MG996R Servo180°, 10kg-cm torque3Platform actuation~$1516x2 I2C LCDBlue backlight1User interface~$4Push ButtonsTactile switches5Navigation~$2USB Webcam720p minimum1Computer vision~$10Power Supply5V 3A / 7.4V Battery1System power~$5Hexagonal PlatformLightweight1Ball surfaceDIY
Total Cost: ~$30-40

🎯 Detailed Component Advantages
<details>
<summary><b>🔧 Click to expand: Why Each Phase 2 Component?</b></summary>
Stepper Motors (NEMA 17) - Phase 2

✅ Precise micro-stepping control (up to 1/32 micro-steps = 6400 steps/rev)
✅ Higher torque at low speeds (better for slow, controlled movements)
✅ Zero position drift when holding (critical for stability)
✅ Predictable, repeatable positioning (±0.05° accuracy)
✅ Better for continuous small adjustments
✅ No feedback loop needed for position holding
❌ Requires stepper drivers (adds cost and complexity)
❌ Higher power consumption
❌ More expensive (~3x servo cost)

Servo Motors (MG996R) - Phase 1

✅ Cost-effective (~$5 each vs ~$15 for steppers+driver)
✅ Simple 3-wire connection (signal, power, ground)
✅ Built-in position feedback and control
✅ Lower power consumption
✅ Good enough for learning and demonstrations
❌ Position drift under load (±1-2°)
❌ Less precise (analog feedback limitations)
❌ Non-linear response characteristics
❌ Limited to 180° or 270° rotation range

TB6600 Stepper Drivers

✅ Smooth micro-stepping operation
✅ Adjustable current control (protects motors)
✅ Built-in protection circuits
✅ Wide voltage range (9-42V)
✅ Professional-grade reliability

SMPS vs Battery

✅ Consistent voltage output
✅ No voltage drop during operation
✅ No recharging downtime
✅ Higher current capacity
✅ Better for long testing sessions

TFT LCD vs 16x2 Character LCD

✅ Graphical interface capabilities
✅ Real-time plots and visualizations
✅ Color-coded status indicators
✅ More intuitive user experience
✅ Better data presentation

Rotary Encoder vs Push Buttons

✅ Faster menu navigation
✅ Single control for all navigation
✅ More ergonomic interaction
✅ Integrated push button for selection
✅ Cleaner wiring and interface

Dual ESP32 Architecture

✅ Dedicated motor control processor
✅ Separate UI/encoder handling
✅ No performance interference
✅ Faster control loop execution (50Hz vs 20Hz)
✅ Modular system design

</details>

🏛️ Project Structure
project-astra/
├── 📁 Phase1/                          # Simulation & Development Phase
│   ├── README.md                      # Phase 1 documentation
│   ├── 📁 simulation/                 # Algorithm testing
│   ├── 📁 docs/                       # Research papers
│   ├── 📁 ai_models/                  # AI algorithms
│   └── 📁 analysis/                   # Performance analysis
│
├── 📁 Phase2/                          # Hardware Implementation Phase
│   ├── README.md                      # Phase 2 documentation  
│   ├── 📁 firmware/                   # ESP32 Arduino code
│   │   ├── motor_controller/         # ESP32 #1 - Stepper control
│   │   └── ui_controller/            # ESP32 #2 - Display & encoder
│   ├── 📁 visualization/              # Python real-time plotting
│   ├── 📁 hardware/                   # Physical system design
│   │   ├── schematics/               # Electrical diagrams
│   │   ├── pcb/                      # PCB designs (optional)
│   │   └── mechanical/               # 3D models & assembly
│   └── 📁 web_interface/              # Optional web dashboard
│
├── 📁 shared/                          # Common resources
│   ├── requirements.txt               # Python dependencies
│   ├── config.yaml                   # System configuration
│   └── LICENSE                       # MIT License
│
└── README.md                          # This file

🚀 Quick Start
Phase 1: Simulation & Development
bash# Navigate to Phase 1
cd Phase1/

# Follow Phase 1 specific instructions
# See Phase1/README.md for detailed setup
Phase 2: Hardware Implementation
bash# Navigate to Phase 2 (Available Soon)
cd Phase2/

# Hardware setup and firmware upload
# See Phase2/README.md for detailed instructions

👥 Team Members — IOE 2078 Batch
NameSpecializationKey ContributionsGitHubSneha YadavAI/ML & SoftwareAI algorithms, predictive compensation, visualization@Sneha-YadavRudra KhatriHardware IntegrationStepper control, platform design, system integration@Roshan-khatri78Bishakha PokhrelTesting & DocumentationESP32 firmware, adaptive PID, optimization@bishakhapokhrelSusant DahalEmbedded SystemsSystem validation, UI development, documentation@sushantdahal
Institution: Purwanchal Campus, Dharan-08, Sunsari
Program: Bachelor of Engineering (BEI) - Electronics & Communication
Academic Year: 2078 Batch
Project Supervisor: Bishnu Chaudhary
Project Duration: April 2024 - January 2026

🎓 Academic Context
Course Integration

Control Systems Engineering: Advanced PID control with AI enhancement
Artificial Intelligence: Machine learning algorithms for system optimization
Embedded Systems: Dual ESP32 programming and real-time control
Computer Vision: OpenCV implementation for ball tracking
Signal Processing: Error analysis and system identification
Power Electronics: Stepper motor control and driver circuits

Research Contributions

Novel AI-PID Architecture: First integration of predictive compensation in ball balancing
Real-time Learning: Online adaptation of control parameters
Multi-modal Visualization: Comprehensive system state visualization on TFT display
Performance Benchmarking: Quantitative AI enhancement validation
Dual-Processor Control: Distributed processing for improved real-time performance


📊 Expected Performance Metrics
Target Specifications
MetricTarget ValueControl Loop Frequency50Hz (20ms cycle time)Ball Position Accuracy±0.005 units from targetMotor Step Resolution6400 steps/revolution (1/32 micro-stepping)System Response Time<50ms for position changesAI Learning Convergence200 samples (~4 seconds at 50Hz)System Stabilization5-30 seconds (adaptive)Display Update Rate20Hz smooth visualization
AI Enhancement Goals
GoalTarget ImprovementError Reduction40-70% vs traditional PIDStability Improvement3x faster convergenceDisturbance Rejection80% reduction in systematic errorsOvershoot Reduction60% less overshoot during transitionsPosition Repeatability±0.003 units with stepper motors

🚀 Future Development Roadmap
Short-term Goals (Phase 2)

 Complete dual ESP32 firmware implementation
 TB6600 stepper driver integration and tuning
 TFT display graphical interface development
 Rotary encoder menu system implementation
 Inter-ESP32 communication protocol
 Real-time AI integration
 Performance validation

Long-term Vision

 Deep Reinforcement Learning: Neural network-based control
 Multi-ball Tracking: Simultaneous control of multiple objects
 IoT Integration: Cloud-based monitoring and control
 Mobile App Interface: Smartphone remote operation
 Advanced Visualization: Real-time 3D platform state
 Adaptive Micro-stepping: Dynamic step resolution optimization


🤝 Contributing
We welcome contributions from the academic and open-source communities!
How to Contribute

Fork the repository
Choose Phase: Decide whether to contribute to Phase 1 (simulation) or Phase 2 (hardware)
Create feature branch: git checkout -b feature/AmazingFeature
Implement enhancement with proper documentation
Test thoroughly
Commit changes: git commit -m 'Add some AmazingFeature'
Push to branch: git push origin feature/AmazingFeature
Submit Pull Request

Contribution Areas

🤖 AI Algorithm Improvements: Enhanced learning algorithms
📊 Simulation Enhancement: Better physics modeling
🔧 Hardware Design: Mechanical improvements, stepper optimization
💻 Firmware Development: ESP32 code enhancements
🎨 UI/UX Design: TFT display interfaces
📚 Documentation: Tutorials and guides
🧪 Testing: Validation scripts


📞 Support & Contact
Technical Support

🐛 Bug Reports: GitHub Issues
💬 Discussions: GitHub Discussions
📧 Email Support: rudrakhatri456@gmail.com

Academic Collaboration

🏛️ Institution: Purwanchal Campus, Institute of Engineering
📍 Location: Dharan-08, Sunsari, Nepal
🌐 University: Tribhuvan University, IOE


📜 License & Citation
MIT License
This project is licensed under the MIT License - see LICENSE file for details.
Academic Citation
bibtex@misc{astra2024,
  title={ASTRA: AI-Enhanced Ball Balancing Hexabot with Adaptive PID Control},
  author={Khatri, Rudra and Yadav, Sneha and Pokhrel, Bishakha and Dahal, Sushant},
  year={2024},
  institution={Purwanchal Campus, IOE, Tribhuvan University},
  address={Dharan, Nepal},
  note={Final Year BEI Project - 2078 Batch}
}

🙏 Acknowledgments
Special Thanks

🏛️ Purwanchal Campus Faculty for guidance and laboratory access
👨‍🏫 Project Supervisor (Bishnu Chaudhary) for technical mentorship
🧑‍💻 Open Source Community for libraries and tools
🏭 Industry Partners for hardware support and testing facilities


<div align="center">
🌟 Project ASTRA represents the future of intelligent control systems 🌟
Combining Classical Control Theory with Modern Artificial Intelligence
🚀 From Chaos to Perfect Control through AI 🚀
Made with ❤️ by IOE 2078 Batch Students

⭐ If this project inspires you, please give it a star! ⭐

[Project ASTRA - Where Intelligence Meets Control]
</div>
