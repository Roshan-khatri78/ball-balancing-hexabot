# API Reference - Ball Balancing Hexabot

## Arduino Firmware API

### Core Functions

#### Servo Control Module
```cpp
// servo_control.h
class ServoControl {
public:
    ServoControl();
    void initialize();
    void setAngle(int servoIndex, int angle);
    int getAngle(int servoIndex);
    void emergencyStop();
    void centerAll();
    bool isWithinLimits(int angle);
    
private:
    Servo servos[3];
    int currentAngles[3];
    const int servoLimits[2] = {-90, 90};
};
```

#### LCD Interface Module  
```cpp
// lcd_interface.h
class LCDInterface {
public:
    LCDInterface();
    void initialize();
    void displayMenu(int selectedOption);
    void displayStatus(String message);
    void displayPIDValues(float kp, float ki, float kd);
    int readButtonInput();
    void clearDisplay();
    
private:
    LiquidCrystal_I2C lcd;
    int lastButtonState[5];
    unsigned long lastDebounceTime[5];
};
```

#### Serial Communication Module
```cpp
// serial_comm.h
class SerialComm {
public:
    SerialComm(int baudRate = 115200);
    void initialize();
    bool available();
    String readCommand();
    void sendResponse(String response);
    void sendStatus(int servoAngles[3], String mode);
    void sendError(String errorMsg);
    
private:
    String inputBuffer;
    bool parseCommand(String command, String& cmd, float values[]);
};
```

### Command Protocol

#### Commands (Python → ESP32)
| Command | Format | Description | Response |
|---------|--------|-------------|----------|
| `SERVO` | `SERVO,X,Y,Z` | Set servo angles (-90 to 90) | `OK,X,Y,Z` |
| `STATUS` | `STATUS` | Request system status | `STATUS,mode,x,y,z` |
| `RESET` | `RESET` | Reset to center position | `OK,RESET` |
| `EMERGENCY` | `EMERGENCY` | Emergency stop all servos | `OK,EMERGENCY` |
| `PID` | `PID,KP,KI,KD` | Set PID parameters | `OK,PID` |
| `MODE` | `MODE,AUTO/MANUAL` | Set control mode | `OK,MODE,X` |

#### Responses (ESP32 → Python)
| Response | Format | Description |
|----------|--------|-------------|
| `OK` | `OK,command,value` | Command executed successfully |
| `ERROR` | `ERROR,message` | Error occurred |
| `STATUS` | `STATUS,mode,x,y,z` | Current system status |

## Python Software API

### Ball Tracker Class

```python
class BallTracker:
    """Real-time ball detection and tracking using OpenCV"""
    
    def __init__(self, camera_index=0, resolution=(640, 480)):
        """
        Initialize ball tracker
        
        Args:
            camera_index (int): Camera device index
            resolution (tuple): Camera resolution (width, height)
        """
        
    def detect_ball(self, frame):
        """
        Detect ball in given frame
        
        Args:
            frame (numpy.ndarray): Input image frame
            
        Returns:
            tuple: (x, y, radius) if ball found, None otherwise
        """
        
    def get_normalized_position(self):
        """
        Get ball position normalized to [-1, 1] range
        
        Returns:
            tuple: (x, y) normalized coordinates
        """
        
    def calibrate_colors(self, ball_color='orange'):
        """
        Calibrate color detection for specific ball
        
        Args:
            ball_color (str): Ball color name
        """
        
    def set_roi(self, x, y, width, height):
        """Set region of interest for ball detection"""
        
    def get_tracking_quality(self):
        """Get current tracking quality score (0-1)"""
```

### PID Controller Class

```python
class PIDController:
    """PID Controller implementation for ball balancing"""
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=(-90, 90)):
        """
        Initialize PID controller
        
        Args:
            kp (float): Proportional gain
            ki (float): Integral gain  
            kd (float): Derivative gain
            output_limits (tuple): Min and max output values
        """
        
    def update(self, setpoint, current_value, dt):
        """
        Calculate PID output
        
        Args:
            setpoint (float): Desired value
            current_value (float): Current measured value
            dt (float): Time step since last update
            
        Returns:
            float: PID output value
        """
        
    def reset(self):
        """Reset PID controller internal state"""
        
    def auto_tune(self, target_function, iterations=100):
        """
        Auto-tune PID parameters using Ziegler-Nichols method
        
        Args:
            target_function: Function to evaluate system response
            iterations (int): Number of tuning iterations
        """
        
    def get_components(self):
        """
        Get individual PID components
        
        Returns:
            tuple: (proportional, integral, derivative) values
        """
```

### Serial Handler Class

```python
class SerialHandler:
    """Handle serial communication with ESP32"""
    
    def __init__(self, port, baudrate=115200, timeout=1):
        """
        Initialize serial connection
        
        Args:
            port (str): Serial port name
            baudrate (int): Communication speed
            timeout (float): Read timeout in seconds
        """
        
    def connect(self):
        """Establish serial connection"""
        
    def disconnect(self):
        """Close serial connection"""
        
    def send_servo_command(self, x_angle, y_angle, z_angle):
        """
        Send servo position command
        
        Args:
            x_angle (int): X-axis servo angle (-90 to 90)
            y_angle (int): Y-axis servo angle (-90 to 90) 
            z_angle (int): Z-axis servo angle (-90 to 90)
            
        Returns:
            bool: True if command sent successfully
        """
        
    def get_system_status(self):
        """
        Request system status from ESP32
        
        Returns:
            dict: System status information
        """
        
    def emergency_stop(self):
        """Send emergency stop command to ESP32"""
        
    def set_pid_parameters(self, kp, ki, kd):
        """Send PID parameters to ESP32"""
```

### Main Controller Class

```python
class MainController:
    """Main application controller coordinating all components"""
    
    def __init__(self, config_file='config.yaml'):
        """
        Initialize main controller
        
        Args:
            config_file (str): Configuration file path
        """
        
    def start_balancing(self):
        """Start automatic ball balancing"""
        
    def stop_balancing(self):
        """Stop ball balancing and center platform"""
        
    def run_calibration(self):
        """Run system calibration routine"""
        
    def manual_control(self, x_input, y_input):
        """
        Manual platform control
        
        Args:
            x_input (float): X-axis input (-1 to 1)
            y_input (float): Y-axis input (-1 to 1)
        """
        
    def get_performance_metrics(self):
        """
        Get system performance metrics
        
        Returns:
            dict: Performance statistics
        """
```

## Simulation API

### Ball Physics Simulation

```python
class BallPhysics:
    """Simulate ball physics on tilting platform"""
    
    def __init__(self, platform_radius=0.15, ball_mass=0.02):
        """
        Initialize physics simulation
        
        Args:
            platform_radius (float): Platform radius in meters
            ball_mass (float): Ball mass in kg
        """
        
    def update(self, platform_tilt_x, platform_tilt_y, dt):
        """
        Update ball position based on platform tilt
        
        Args:
            platform_tilt_x (float): Platform tilt X in radians
            platform_tilt_y (float): Platform tilt Y in radians
            dt (float): Time step in seconds
        """
        
    def get_ball_position(self):
        """
        Get current ball position
        
        Returns:
            tuple: (x, y) position in meters
        """
        
    def reset_ball(self, x=0, y=0):
        """Reset ball to specified position"""
        
    def apply_disturbance(self, force_x, force_y):
        """Apply external disturbance to ball"""
```

### Platform Dynamics

```python
class PlatformDynamics:
    """Simulate hexagonal platform servo dynamics"""
    
    def __init__(self, servo_response_time=0.06):
        """
        Initialize platform dynamics
        
        Args:
            servo_response_time (float): Servo response time in seconds
        """
        
    def set_target_angles(self, angles):
        """Set target servo angles"""
        
    def update(self, dt):
        """Update servo positions toward targets"""
        
    def get_platform_tilt(self):
        """
        Calculate platform tilt from servo positions
        
        Returns:
            tuple: (tilt_x, tilt_y) in radians
        """
        
    def get_servo_angles(self):
        """Get current servo angles"""
```

## Configuration API

### Configuration File Format (YAML)

```yaml
# config.yaml
camera:
  index: 0
  resolution: [640, 480]
  fps: 30
  
ball_detection:
  color: 'orange'
  min_radius: 10
  max_radius: 100
  
pid_controllers:
  x_axis:
    kp: 2.0
    ki: 0.1
    kd: 0.5
  y_axis:
    kp: 2.0
    ki: 0.1
    kd: 0.5
    
serial:
  port: 'COM3'  # Windows
  # port: '/dev/ttyUSB0'  # Linux
  baudrate: 115200
  timeout: 1.0
  
platform:
  servo_limits: [-90, 90]
  response_time: 0.06
  safety_margin: 5
  
simulation:
  enable: false
  physics_timestep: 0.01
  visualization_fps: 30
```

### Error Codes

| Code | Category | Description | Solution |
|------|----------|-------------|----------|
| E001 | Camera | Camera not found | Check camera connection |
| E002 | Camera | Frame capture failed | Restart camera |
| E003 | Serial | Port not available | Check port name/permissions |
| E004 | Serial | Communication timeout | Check ESP32 connection |
| E005 | Servo | Angle out of range | Verify servo limits |
| E006 | Ball | Ball not detected | Check lighting/ball color |
| E007 | System | Emergency stop activated | Reset system |
| E008 | PID | Controller instability | Tune PID parameters |

## Usage Examples

### Basic Integration Example

```python
from software.main_controller import MainController
from software.config import load_config

# Load configuration
config = load_config('config.yaml')

# Initialize controller
controller = MainController(config)

# Start balancing
try:
    controller.start_balancing()
    
    # Run for 60 seconds
    time.sleep(60)
    
finally:
    controller.stop_balancing()
    print("Balancing stopped safely")
```

### Advanced Control Example

```python
import numpy as np
from software.ball_tracker import BallTracker
from software.pid_controller import PIDController
from software.serial_handler import SerialHandler

# Initialize components
tracker = BallTracker(camera_index=0)
pid_x = PIDController(kp=2.5, ki=0.2, kd=0.8)
pid_y = PIDController(kp=2.5, ki=0.2, kd=0.8)
serial_comm = SerialHandler('COM3')

# Control loop with error handling
try:
    last_time = time.time()
    
    while True:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        # Get ball position
        ball_pos = tracker.get_normalized_position()
        
        if ball_pos is not None:
            x, y = ball_pos
            
            # PID control (target = center)
            control_x = pid_x.update(0, x, dt)
            control_y = pid_y.update(0, y, dt)
            
            # Send to hardware
            success = serial_comm.send_servo_command(
                int(control_x), int(control_y), 0
            )
            
            if not success:
                print("Communication error")
                break
                
        else:
            # Ball lost - center platform
            serial_comm.send_servo_command(0, 0, 0)
            
        # Control frequency
        time.sleep(0.02)  # 50 Hz
        
except KeyboardInterrupt:
    print("Stopping...")
    
finally:
    serial_comm.emergency_stop()
    tracker.release()
```
