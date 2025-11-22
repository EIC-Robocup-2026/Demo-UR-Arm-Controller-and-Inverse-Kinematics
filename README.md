# Demo Arm Controller and Inverse Kinematic

A real-time keyboard controller for a UR-style robotic arm with inverse kinematics support. This project enables intuitive control of a 6-DOF robotic arm using keyboard inputs and is designed to interface with ESP32 via PySerial for wireless servo control.

![Demo Arm](Demo%20Arm%20Image.png)

## Overview

This project provides a real-time keyboard controller for controlling a UR (Universal Robots) style robotic arm. The system captures keyboard inputs and translates them into directional vectors and gripper commands, which can be used for inverse kinematics calculations and servo motor control.

## Dependencies

The project requires the following Python packages:

- **numpy** - For numerical computations and vector operations
- **sympy** - For symbolic and differential computations
- **pynput** - For capturing keyboard input events in real-time
- **serial** - For serial communication with ESP32 microcontroller
- **matplotlib** - For real-time visualization of the arm's position and orientation

Install dependencies using:
```bash
pip install numpy pynput serial matplotlib sympy
```

## Project Structure

```
Demo-Arm-Controller-and-Inverse-Kinematic/
├── src/
│   ├── main.py              # Main application entry point
│   ├── keyboard_input.py    # URMController class for keyboard handling
│   └── parameter.py         # Robot arm physical parameters and constants
├── DEMO ARM EXPORT.stp      # 3D CAD model of the arm (STEP format)
└── Demo Arm Image.png       # Reference image of the arm design
```

## Key Components

### 1. URMController (`keyboard_input.py`)

The core controller class that handles keyboard input and maintains the robot's state.

**Features:**
- Real-time keyboard input capture using `pynput`
- Continuous directional control (X, Y, Z axes)
- Gripper angle control (0-90 degrees)
- Non-blocking listener for smooth operation

**Control Mapping:**
- **W/A/S/D** - Move in X/Y plane (forward/left/back/right)
- **Space** - Move up (+Z axis)
- **Shift** - Move down (-Z axis)
- **Left Arrow** - Close gripper (decrease angle)
- **Right Arrow** - Open gripper (increase angle)
- **ESC** - Exit program

**Key Methods:**
- `start()` - Initialize keyboard listeners
- `returnDirection()` - Get current direction vector [x, y, z]
- `returnGripperValue()` - Get current gripper angle (0-90°)
- `stop()` - Cleanup and stop listeners

### 2. Robot Parameters (`parameter.py`)

Contains physical dimensions and initial configuration for the UR-style arm.

**Link Lengths (mm):**
- l1 = 172 mm (base height)
- l2 = 350 mm (upper arm)
- l3 = 350 mm (forearm)
- l4 = 95 mm (wrist to end effector)

**Widths (mm):**
- a1 = 96.05 mm
- a2 = 95.8 mm
- a3 = 95 mm
- a4 = 50 mm

### 3. Main Application (`main.py`)

Demonstrates basic usage of the URMController with a simple control loop.

**Functionality:**
- Initializes the controller
- Continuously polls for direction and gripper values
- Prints current state (20 Hz update rate)
- Exits on ESC key press

## Usage

### Basic Usage

Run the main application:
```bash
python src/main.py
```

### Integration Example

```python
from keyboard_input import URMController
import time

# Initialize controller
controller = URMController()
controller.start()  # Non-blocking start

# Main control loop
while controller.running:
    # Get current state
    direction = controller.returnDirection()  # Returns [x, y, z] numpy array
    gripper = controller.returnGripperValue()  # Returns angle 0-90
    
    print(f"Direction: {direction}, Gripper: {gripper}")
    
    # TODO: Implement inverse kinematics here
    # TODO: Send commands to ESP32 via serial
    
    # Exit on ESC
    if not controller.running:
        break
    
    time.sleep(0.05)  # 20 Hz update rate

controller.stop()
```

## Future Development

The project is designed to integrate with:
- **Inverse Kinematics Engine** - Convert Cartesian directions to joint angles
- **ESP32 Communication** - PySerial integration for wireless servo control
- **Real-time Feedback** - Sensor data from physical arm

## Hardware Requirements

- 6-DOF robotic arm with servo motors
- ESP32 microcontroller for wireless control
- Power supply for servos
- USB cable or serial adapter for ESP32 communication

## Notes

- Direction vectors are normalized and suitable for velocity-based control
- Gripper angle range is 0-90 degrees with 1-degree increments
- The controller uses a non-blocking design for integration with other systems
- Physical dimensions match the CAD model provided (DEMO ARM EXPORT.stp)

## License

This project is part of the EIC RoboCup 2026 initiative.