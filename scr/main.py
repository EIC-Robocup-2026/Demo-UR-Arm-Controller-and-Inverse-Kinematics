from keyboard_input import URMController
import keyboard_input 
import time

# Initialize controller once
controller = URMController()
controller.start()  # Starts background listener (non-blocking)

# Now run your other code
while True:
    # Get current direction and gripper values anytime
    direction = controller.returnDirection()
    gripper = controller.returnGripperValue()
    
    print(f"Direction: {direction}, Gripper: {gripper}")
    
    # Use these values for your inverse kinematics or other logic here
    
    # Check if ESC was pressed to exit
    if not controller.running:
        break
    
    time.sleep(0.05)

controller.stop()