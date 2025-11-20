from keyboard_input import URMController
import parameter
import time

# Initialize controller once
controller = URMController()
controller.start()  # Starts background listener (non-blocking)

# Initialize UR Arm parameters
parameter = parameter.URArmParameter()
print("UR Arm parameters initialized.")

# Now run your other code
while True:
    # Get current direction and gripper values anytime
    direction = controller.returnDirection()
    gripper = controller.returnGripperValue()
    parameter.move_by(direction)
    
    # Get current state
    end_pos = parameter.get_end_effector_position()
    thetas = parameter.get_current_thetas()
    
    # Print on one line: thetas and end position
    theta_str = " | ".join([f"θ{i+1}: {float(thetas[i]):.4f}" for i in range(6)])
    pos_str = f"Pos: [{float(end_pos[0]):.2f}, {float(end_pos[1]):.2f}, {float(end_pos[2]):.2f}]"
    print(f"{theta_str} | {pos_str}")
    
    if not controller.running:
        break
    
    time.sleep(0.05)

controller.stop()