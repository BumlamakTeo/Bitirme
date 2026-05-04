import pygame
import time

def main():
    # Initialize pygame and its joystick module
    pygame.init()
    pygame.joystick.init()
    
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("Error: No controllers found. Please ensure your 8BitDo 2.4G dongle is plugged in and the controller is turned on.")
        pygame.quit()
        return
        
    print(f"Found {joystick_count} controller(s).")
    
    # Connect to the first available controller
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    name = joystick.get_name()
    axes_count = joystick.get_numaxes()
    buttons_count = joystick.get_numbuttons()
    
    print(f"Connected to: {name}")
    print(f"Detected {axes_count} axes and {buttons_count} buttons.")
    print("\nReading live controller values... (Press Ctrl+C to quit)\n")
    
    try:
        while True:
            # We must pump the pygame event queue to get updated controller states
            pygame.event.pump()
            
            # For a standard Xbox-style controller in Windows:
            # Axis 0: Left Stick X (Negative = Left, Positive = Right)
            # Axis 1: Left Stick Y (Negative = Up, Positive = Down)
            # Axis 2: Left Trigger (or Right Stick X depending on driver)
            # Axis 3: Right Stick X (or Right Stick Y)
            # Axis 4: Right Stick Y
            # Axis 5: Right Trigger
            
            output_string = ""
            for i in range(axes_count):
                # Format to 3 decimal places for readability
                val = joystick.get_axis(i)
                output_string += f"Axis {i}: {val:>6.3f}   "
                
            # Print over the same line so it doesn't spam the terminal vertically
            print(f"\r{output_string}", end="")
            time.sleep(0.05)  # 20 Hz read rate
            
    except KeyboardInterrupt:
        print("\n\nExiting controller reader...")
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()
