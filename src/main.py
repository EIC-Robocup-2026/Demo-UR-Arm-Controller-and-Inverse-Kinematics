from keyboard_input import URMController
import parameter
import const as c
import time
import serial
import math



# Initialize serial communication with ESP32 on COM7
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
    gripper_roll_direction = controller.returnGripperRollDirection()

    parameter.move_by(direction)
    parameter.rotate_gripper(gripper_roll_direction)
    
    # Get current state
    end_pos = parameter.get_end_effector_position()
    thetas = parameter.get_current_thetas()

    values = []
    for i in range(6):
        theta_deg = math.degrees(float(thetas[i])) - c.THETA_OFFSET_ANGLE[i]
        values.append(f"{theta_deg:.4f}")
    gripper_val = gripper - c.THETA_OFFSET_ANGLE[6]
    values.append(f"{gripper_val:.4f}")
    print(",".join(values))
    
    if not controller.running:
        break
    
    # Print on one line: thetas, end position, gripper, and gripper roll direction
    # theta_str = " | ".join([f"θ{i+1}: {float(thetas[i]):.4f}" for i in range(6)])
    # pos_str = f"Pos: [{float(end_pos[0]):.2f}, {float(end_pos[1]):.2f}, {float(end_pos[2]):.2f}]"
    # gripper_str = f"Gripper: {gripper:.1f}°"
   
    # print(f"{theta_str} | {pos_str} | {gripper_str}")
    # print(f"Direction: {direction} | Roll: {gripper_roll_direction}")

    # Convert theta 1-6 and gripper angle to int, subtract offset, and print

    time.sleep(0.05)

controller.stop()