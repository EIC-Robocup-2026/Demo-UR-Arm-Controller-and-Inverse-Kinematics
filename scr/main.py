from keyboard_input import URMController
import parameter
import time
import serial



# Initialize serial communication with ESP32 on COM7 (optional)
ser = None
try:
    ser = serial.Serial(port='COM7', baudrate=115200, timeout=1)
    print(f"Serial port opened: {ser.name}")
except Exception as e:
    print(f"Serial port not available (continuing without): {e}")

# Initialize controller once
controller = URMController()
controller.start()  # Starts background listener (non-blocking)

# Initialize UR Arm parameters
parameter = parameter.URArmParameter()
print("UR Arm parameters initialized.")

while True:
    # Get current direction and gripper values
    direction = controller.returnDirection()
    gripper = controller.returnGripperValue()
    parameter.move_by(direction)
    
    # Get current state
    end_pos = parameter.get_end_effector_position()
    thetas = parameter.get_current_thetas()
    
    # Print on one line: thetas, end position, and gripper
    theta_str = " | ".join([f"θ{i+1}: {float(thetas[i]):.4f}" for i in range(6)])
    pos_str = f"Pos: [{float(end_pos[0]):.2f}, {float(end_pos[1]):.2f}, {float(end_pos[2]):.2f}]"
    gripper_str = f"Gripper: {gripper:.1f}°"
    print(f"{theta_str} | {pos_str} | {gripper_str}")
    
    if not controller.running:
        break
    
    time.sleep(0.05)

controller.stop()