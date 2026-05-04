import pygame
import time
import sys
from pantilt import PanTiltAxis, COM_PORT, BAUD_RATE
from driver import MksServoBus

def main():
    print("Initializing Joystick...")
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        print("No controllers found. Make sure your 8BitDo controller is connected.")
        sys.exit(1)
        
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Connected to controller: {joystick.get_name()}")

    print(f"Connecting to Pan-Tilt System on {COM_PORT}...")
    try:
        # We use a lower timeout (0.2s) for responsive joystick control, unlike the blocking absolute mode
        bus = MksServoBus(port=COM_PORT, baudrate=BAUD_RATE, timeout=0.2, debug=False)
    except Exception as e:
        print(f"Failed to open {COM_PORT}: {e}")
        sys.exit(1)

    # Initialize the axes using the PanTiltAxis logic
    tilt_axis = PanTiltAxis(name="TILT", bus=bus, address=0xE1, gear_ratio=4.0, min_angle=-30.0, max_angle=80.0)
    pan_axis = PanTiltAxis(name="PAN", bus=bus, address=0xE0, gear_ratio=5.0, min_angle=-180.0, max_angle=180.0)

    try:
        tilt_axis.init_motor()
        time.sleep(0.1)
        pan_axis.init_motor()
    except Exception as e:
        print(f"Failed to initialize motors: {e}")
        sys.exit(1)
        
    print("\n=========================================")
    print("        Joystick Control Active          ")
    print("=========================================")
    print("Left Stick Y (Up/Down):   TILT")
    print("Right Stick X (L/R):      PAN")
    print("Press Ctrl+C to quit.")
    print("=========================================\n")
    
    try:
        while True:
            # Poll the controller
            pygame.event.pump()
            
            # Read joystick axes based on your logs
            # Axis 1: Left Stick Y (Negative = Up, Positive = Down)
            # Axis 3: Right Stick X (Negative = Left, Positive = Right)
            
            # We invert Axis 1 so pushing UP (negative axis value) results in a POSITIVE speed ratio (Tilt Up)
            tilt_speed = -joystick.get_axis(1)
            
            # Right Stick X drives Pan normally
            pan_speed = joystick.get_axis(3)
            
            # Stream continuous velocity commands to the motor controller.
            # This method calculates limits by reading the real-time encoder before moving!
            tilt_axis.set_velocity(tilt_speed)
            pan_axis.set_velocity(pan_speed)
            
            # Print live feedback on the same line
            t_angle = tilt_axis.current_angle
            p_angle = pan_axis.current_angle
            print(f"\rTILT: {t_angle:>6.1f}° | PAN: {p_angle:>6.1f}°  ", end="")
            
            # ~20Hz loop for smooth tracking without overwhelming the UART
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\n\nExiting Joystick Control...")
    finally:
        print("Stopping motors...")
        try: tilt_axis.motor.stop()
        except: pass
        try: pan_axis.motor.stop()
        except: pass
        
        bus.close()
        pygame.quit()

if __name__ == "__main__":
    main()
