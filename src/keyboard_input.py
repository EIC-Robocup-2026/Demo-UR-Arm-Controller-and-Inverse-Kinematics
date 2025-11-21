import sympy as sp
import const as c
from pynput import keyboard
import time

class URMController:
    """Controller for UR Arm using pynput keyboard input"""
    
    def __init__(self):
        # Direction vector [x, y, z]
        self.direction = sp.Matrix([0.0, 0.0, 0.0])
        
        # Gripper angle (0-90 degrees)
        self.gripper_angle = c.GRIPPER_INITIAL_ANGLE
        
        self.gripper_roll_direction = 0  # -1 for clockwise, +1 for anti clockwise, 0 for no rotation

        # Running state
        self.running = True
        
        # Key states to track continuous movement
        self.keys_pressed = {
            'w': False,
            'a': False,
            's': False,
            'd': False,
            'shift': False,
            'space': False,
            'left': False,
            'right': False,
            'up': False,
            'down': False
        }
        
        # Setup listeners
        self.keyboard_listener = keyboard.Listener(
            on_press=self.on_key_press,
            on_release=self.on_key_release
        )
    
    def on_key_press(self, key):
        """Handle keyboard press events"""
        try:
            # Character keys (w, a, s, d, r) - handle both lowercase and uppercase
            if hasattr(key, 'char'):
                char = key.char.lower() if key.char else None
                if char in self.keys_pressed:
                    self.keys_pressed[char] = True
            
            # Special keys
            elif key == keyboard.Key.shift_l or key == keyboard.Key.shift_r:
                self.keys_pressed['shift'] = True
            elif key == keyboard.Key.space:
                self.keys_pressed['space'] = True
            elif key == keyboard.Key.left:
                self.keys_pressed['left'] = True
            elif key == keyboard.Key.right:
                self.keys_pressed['right'] = True
            elif key == keyboard.Key.up:
                self.keys_pressed['up'] = True
            elif key == keyboard.Key.down:
                self.keys_pressed['down'] = True
            elif key == keyboard.Key.esc:
                self.running = False
                return False
        
        except AttributeError:
            pass
    
    def on_key_release(self, key):
        """Handle keyboard release events"""
        try:
            if hasattr(key, 'char'):
                char = key.char.lower() if key.char else None
                if char in self.keys_pressed:
                    self.keys_pressed[char] = False
            elif key == keyboard.Key.shift_l or key == keyboard.Key.shift_r:
                self.keys_pressed['shift'] = False
            elif key == keyboard.Key.space:
                self.keys_pressed['space'] = False
            elif key == keyboard.Key.left:
                self.keys_pressed['left'] = False
            elif key == keyboard.Key.right:
                self.keys_pressed['right'] = False
            elif key == keyboard.Key.up:
                self.keys_pressed['up'] = False
            elif key == keyboard.Key.down:
                self.keys_pressed['down'] = False
        
        except AttributeError:
            pass
    
    def update_gripper(self):
        """Update gripper angle and roll direction based on arrow key states"""
        # Gripper angle control (up/down arrows)
        if self.keys_pressed['up']:
            # Up arrow: open gripper (increase angle)
            self.gripper_angle = min(self.gripper_angle + 1, c.GRIPPER_MAX_ANGLE)
        if self.keys_pressed['down']:
            # Down arrow: close gripper (decrease angle)
            self.gripper_angle = max(self.gripper_angle - 1, c.GRIPPER_MIN_ANGLE)
        
        # Gripper roll direction control (left/right arrows)
        if self.keys_pressed['right']:
            self.gripper_roll_direction = 1
        elif self.keys_pressed['left']:
            self.gripper_roll_direction = -1
        else:
            self.gripper_roll_direction = 0
    
    def update_direction(self):
        """Update direction vector based on current key states"""
        # Reset direction vector
        self.direction = sp.Matrix([0.0, 0.0, 0.0])
        
        # X-axis control (A/D keys)
        if self.keys_pressed['d']:
            self.direction[0] += 1.0
        if self.keys_pressed['a']:
            self.direction[0] -= 1.0
        
        # Y-axis control (W/S keys)
        if self.keys_pressed['w']:
            self.direction[1] += 1.0
        if self.keys_pressed['s']:
            self.direction[1] -= 1.0
        
        # Z-axis control (Space/Shift keys)
        if self.keys_pressed['space']:
            self.direction[2] += 1.0
        if self.keys_pressed['shift']:
            self.direction[2] -= 1.0
        
        # Update gripper angle
        self.update_gripper()
    
    def start(self):
        """Start the controller listeners"""
        self.keyboard_listener.start()
        print("UR Arm Controller Started")
        print("Controls:")
        print("  W/A/S/D - Move in X/Y plane")
        print("  Space/Shift - Move up/down (Z axis)")
        print("  ↑ (Up Arrow) - Open gripper")
        print("  ↓ (Down Arrow) - Close gripper")
        print("  <- (Left Arrow) - Gripper roll direction -1")
        print("  -> (Right Arrow) - Gripper roll direction +1")
        print("  ESC - Exit")
        print()
    
    def get_state(self):
        """Get current state: (direction_vector, gripper_angle, gripper_roll_direction)"""
        self.update_direction()
        return {
            'direction': self.direction,
            'gripper_angle': self.gripper_angle,
            'gripper_roll_direction': self.gripper_roll_direction
        }
    
    def returnDirection(self):
        """Return the current direction vector"""
        self.update_direction()
        return self.direction
    
    def returnGripperValue(self):
        """Return the current gripper angle value (0-90)"""
        return self.gripper_angle
    
    def returnGripperRollDirection(self):
        """Return the current gripper roll direction (1, 0, or -1)"""
        return self.gripper_roll_direction
    
    def run(self, update_rate=20):
        """Run the controller update loop"""
        self.start()
        
        try:
            while self.running:
                state = self.get_state()
                print(f"Direction: {state['direction']} | Gripper: {state['gripper_angle']:.1f}°")
                print(controller.get_state())
                time.sleep(1.0 / update_rate)
        
        except KeyboardInterrupt:
            print("\nController interrupted")
        
        finally:
            self.stop()
    
    def stop(self):
        """Stop the controller"""
        self.keyboard_listener.stop()
        print("UR Arm Controller Stopped")


if __name__ == "__main__":
    controller = URMController()
    controller.run()
    
