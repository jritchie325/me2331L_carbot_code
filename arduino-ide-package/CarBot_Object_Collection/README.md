# CarBot Object Collection System

An autonomous Arduino-based robot that scans for objects, grabs them with a servo gripper, and delivers them using L-shaped movement patterns.

## Features

- **Autonomous Object Detection**: 360-degree scanning with ultrasonic sensor
- **Precise Movement Control**: Two-stage PID turning system with IMU feedback
- **Servo Gripper**: Smooth object manipulation with customizable positions
- **L-Shaped Delivery Paths**: Alternating delivery patterns for efficiency
- **Memory Optimized**: Runs efficiently on Arduino Uno (40% RAM usage)
- **Debug Controls**: Easy-to-enable diagnostic output

## Hardware Requirements

### Core Components
- **Arduino Uno** (or compatible)
- **MPU6050 IMU** - Motion tracking and heading control
- **HC-SR04 Ultrasonic Sensor** - Object detection
- **TB6612FNG Motor Driver** (or similar dual H-bridge)
- **Standard Servo Motor** (SG90, MG996R, etc.)
- **Two DC Motors** - Differential drive system

### Power Requirements
- 5V for Arduino and sensors
- Separate motor power supply recommended (6-12V depending on motors)
- Servo power consideration (may need separate 5V supply for high-torque servos)

## Pin Connections

### Motor Driver (TB6612FNG)
| Motor Driver Pin | Arduino Pin | Description |
|------------------|-------------|-------------|
| PWMA | 6 | Left Motor Speed (PWM) |
| PWMB | 5 | Right Motor Speed (PWM) |
| AIN1 | 8 | Left Motor Direction |
| AIN2 | 7 | Right Motor Direction |
| STBY | 3 | Standby Control |

### Sensors
| Component | Arduino Pin | Description |
|-----------|-------------|-------------|
| MPU6050 SDA | A4 | I2C Data |
| MPU6050 SCL | A5 | I2C Clock |
| USS Trigger | 13 | Ultrasonic Trigger |
| USS Echo | 12 | Ultrasonic Echo |
| Servo Signal | 10 | Servo Control |

## Required Libraries

Install these libraries through the Arduino IDE Library Manager:

1. **Wire** (built-in) - I2C communication
2. **Servo** (built-in) - Servo motor control
3. **I2Cdevlib-MPU6050** by Jeff Rowberg - IMU interface
4. **PID** by Brett Beauregard - PID control algorithms

### Library Installation
```
Arduino IDE -> Tools -> Manage Libraries
Search for: "I2Cdevlib-MPU6050"
Search for: "PID"
```

## Setup Instructions

1. **Wire Hardware**: Connect components according to pin diagram above
2. **Install Libraries**: Install required libraries in Arduino IDE
3. **Upload Code**: Upload `CarBot_Object_Collection_Arduino.ino` to your Arduino
4. **Calibration**: Keep robot stationary during the ~2 second startup calibration
5. **Operation**: Place objects within 40cm range for detection

## Configuration

### Adjustable Parameters (in main sketch)
```cpp
const int object_grab_range = 5;      // Distance to grab object (cm)
const int angle_increment = 10;       // Scan angle step (degrees)  
const int max_scan_angle = 90;        // Max scan range (degrees)
const int move_duration = 6000;       // Base movement time (ms)
const double move_speed = 2.0;        // Movement speed (m/s)
```

### Servo Positions (in servo_control.h)
```cpp
int servo_arm_open = 140;    // Open position angle
int servo_arm_closed = 44;   // Closed position angle
```

## Operation

### Startup Sequence
1. Serial communication established (115200 baud)
2. Robot object creation and hardware initialization
3. IMU calibration (keep stationary!)
4. System ready message

### Main Loop
1. **Scan Phase**: Robot turns and scans for objects using ultrasonic sensor
2. **Detection**: When object found, robot approaches and grabs it
3. **Delivery**: Executes L-shaped path to delivery area
4. **Release**: Opens gripper and prepares for return journey
5. **Return**: Travels back to pickup area
6. **Repeat**: Continues scanning for more objects

### Debug Output
Enable debug output by commenting out this line in `setup()`:
```cpp
// robot_controller->disableDebug();  // Comment out to keep debug enabled
```

## Troubleshooting

### No Serial Output
- Check baud rate is set to 115200
- Verify correct COM port selection
- Try pressing Arduino reset button

### Robot Not Moving
- Check motor driver connections and power supply
- Verify standby pin (pin 3) connection
- Test with `robot.testMotors()` function

### Poor Object Detection
- Ensure ultrasonic sensor has clear line of sight
- Check trigger/echo pin connections
- Small or curved objects (like ping pong balls) can be challenging
- Adjust `max_sensor_distance` if needed

### Turning Issues
- IMU calibration is critical - keep robot still during startup
- Check I2C connections (SDA/SCL)
- PID parameters can be tuned if needed

## Advanced Tuning

### PID Parameters
Adjust in `move_PID.h`:
```cpp
double v_proportional = 8.0, v_integral = 0.5, v_delta = 0.3;  // Speed PID
double t_proportional = 6, t_integral = 0.5, t_delta = 1.1;    // Turn PID
```

### Motor Balance
If robot curves during straight movement:
```cpp
const float LEFT_MOTOR_COMPENSATION = 1.0;   // Adjust if needed
const float RIGHT_MOTOR_COMPENSATION = 1.0;  // Adjust if needed
```

## File Structure

```
CarBot_Object_Collection_Arduino/
├── CarBot_Object_Collection_Arduino.ino    # Main sketch
├── move_PID.h                               # Robot motion control
├── sonic.h                                  # Ultrasonic sensor
├── servo_control.h                          # Servo gripper control
├── accelerate.h                             # IMU integration
├── README.md                                # This documentation
└── LICENSE                                  # License information
```

## License

This project is open source. See LICENSE file for details.

## Support

For questions or issues:
1. Check troubleshooting section above
2. Verify all hardware connections
3. Test individual components separately
4. Enable debug output for diagnostic information

## Version History

- **v1.0** - Initial Arduino IDE release
  - Complete object collection system
  - Two-stage PID turning control
  - Memory optimized for Arduino Uno
  - Comprehensive documentation