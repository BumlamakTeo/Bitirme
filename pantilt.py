import sys
import time
from driver import MksServoBus

# --- Configuration Constants ---
COM_PORT = "COM14"
BAUD_RATE = 38400

# Global Motor Settings
MSTEP = 16
STEPS_PER_REV = 200
PULSES_PER_MOTOR_REV = MSTEP * STEPS_PER_REV

class PanTiltAxis:
    def __init__(self, name, bus, address, gear_ratio, min_angle, max_angle):
        self.name = name
        self.bus = bus
        self.address = address
        self.gear_ratio = gear_ratio
        self.min_angle = min_angle
        self.max_angle = max_angle
        
        # Calculate how many pulses equal 1 full revolution of the output shaft
        self.pulses_per_shaft_rev = int(PULSES_PER_MOTOR_REV * gear_ratio)
        
        self.current_angle = 0.0
        self.current_pulses = 0
        
        self.motor = bus.motor(address)
        self.zero_carry = 0
        self.zero_value = 0

    def init_motor(self):
        print(f"[{self.name}] Configuring motor at UART address 0x{self.address:02X}...")
        self.motor.set_mode_uart()
        self.motor.set_enable(True)       # Enable motor to lock position
        self.motor.set_mstep(MSTEP)
        self.motor.zero_set_current_position() # Ensure internal motor position starts at 0
        
        # Record the hardware encoder value at this confirmed zero position
        try:
            self.zero_carry, self.zero_value = self.motor.read_encoder()
            print(f"[{self.name}] Recorded absolute zero encoder value: carry={self.zero_carry}, val={self.zero_value}")
        except Exception as e:
            print(f"[{self.name}] Warning: Could not read initial encoder value: {e}")
            self.zero_carry, self.zero_value = 0, 0

    def move_to(self, target_angle, speed_multiplier=1.0):
        # Software Limit Check
        if not (self.min_angle <= target_angle <= self.max_angle):
            print(f"[{self.name}] Error: Target angle {target_angle}° is out of locked physical bounds [{self.min_angle}°, {self.max_angle}°].")
            return
            
        target_pulses = int(round((target_angle / 360.0) * self.pulses_per_shaft_rev))
        delta_pulses = target_pulses - self.current_pulses
        
        if delta_pulses == 0:
            print(f"[{self.name}] Already at {target_angle}°.")
            return
            
        direction_cw = delta_pulses >= 0
        pulses_to_move = abs(delta_pulses)
        
        # --- Dynamic Speed Calculation ---
        abs_delta_angle = abs(target_angle - self.current_angle)
        motor_revs_needed = (abs_delta_angle * self.gear_ratio) / 360.0
        
        TARGET_TRAVEL_TIME = 0.25  # seconds
        desired_rpm = (motor_revs_needed / TARGET_TRAVEL_TIME) * 60.0
        
        # Base limits
        MIN_MOTOR_RPM = 90.0
        MAX_MOTOR_RPM = 400.0
        base_rpm = max(MIN_MOTOR_RPM, min(desired_rpm, MAX_MOTOR_RPM))
        
        motor_rpm = base_rpm * speed_multiplier
        motor_rpm = max(10.0, min(motor_rpm, 800.0)) # Absolute hardware safety bounds
        
        print(f"[{self.name}] Moving to {target_angle}°...")
        print(f" -> Shaft Delta: {target_angle - self.current_angle}°")
        print(f" -> Motor Pulses: {pulses_to_move}, CW={direction_cw}")
        print(f" -> Dynamic Speed: {motor_rpm:.1f} RPM (Base: {base_rpm:.1f} RPM x {speed_multiplier})")
        
        try:
            self.motor.run_pulses(rpm=motor_rpm, pulses=pulses_to_move, mstep=MSTEP, direction_cw=direction_cw)
            self.current_pulses = target_pulses
            self.current_angle = target_angle
            print(f"[{self.name}] Motion complete. Position securely held at {self.current_angle}°.")
        except Exception as e:
            print(f"\n[{self.name}] [CRITICAL ERROR] Error during execution of motion: {e}")
            self.recover_to_zero()

    def get_live_angle(self):
        try:
            curr_carry, curr_value = self.motor.read_encoder()
            # Calculate total encoder difference. CW decreases encoder value.
            encoder_diff = (self.zero_carry * 65536 + self.zero_value) - (curr_carry * 65536 + curr_value)
            motor_revs = encoder_diff / 65536.0
            shaft_revs = motor_revs / self.gear_ratio
            return shaft_revs * 360.0
        except Exception as e:
            # If UART read fails occasionally, return tracked current_angle to prevent false limit triggers
            return self.current_angle

    def set_velocity(self, speed_ratio):
        """
        speed_ratio: -1.0 to 1.0 (from joystick input)
        Positive = CW (increase angle), Negative = CCW (decrease angle)
        """
        if not hasattr(self, 'last_velocity_rpm'):
            self.last_velocity_rpm = 0.0
            self.last_direction = None

        live_angle = self.get_live_angle()
        self.current_angle = live_angle # Update tracked angle
        
        # Deadzone
        if abs(speed_ratio) < 0.05:
            if self.last_velocity_rpm != 0.0:
                self.motor.stop()
                self.last_velocity_rpm = 0.0
            return
            
        # Check software limits
        if speed_ratio > 0 and live_angle >= self.max_angle:
            if self.last_velocity_rpm != 0.0:
                self.motor.stop()
                self.last_velocity_rpm = 0.0
            return
        if speed_ratio < 0 and live_angle <= self.min_angle:
            if self.last_velocity_rpm != 0.0:
                self.motor.stop()
                self.last_velocity_rpm = 0.0
            return
            
        direction_cw = (speed_ratio > 0)
        
        # Max RPM for joystick control
        MAX_JOYSTICK_RPM = 400.0
        MIN_JOYSTICK_RPM = 10.0 
        
        target_rpm = abs(speed_ratio) * MAX_JOYSTICK_RPM
        target_rpm = max(MIN_JOYSTICK_RPM, min(target_rpm, 800.0))
        
        # To avoid flooding UART, only send command if speed changed significantly (> 5 RPM) or direction changed
        if abs(target_rpm - self.last_velocity_rpm) < 5.0 and direction_cw == self.last_direction:
            return
            
        try:
            self.motor.run_constant_speed(rpm=target_rpm, mstep=MSTEP, direction_cw=direction_cw)
            self.last_velocity_rpm = target_rpm
            self.last_direction = direction_cw
        except Exception as e:
            # On error, stop to be safe
            try: self.motor.stop() 
            except: pass
            self.last_velocity_rpm = 0.0

    def recover_to_zero(self):
        print(f"[{self.name}] Initiating emergency recovery: Sending 'Goto 0' command...")
        try:
            self.bus._ser.reset_input_buffer()
            self.motor.zero_goto()
            print(f"[{self.name}] Waiting for motor to return to zero...")
            
            recovered = False
            for i in range(10):
                time.sleep(1.0)
                try:
                    curr_carry, curr_value = self.motor.read_encoder()
                    print(f"[{self.name}] -> Recovery encoder check: carry={curr_carry}, val={curr_value}")
                    if (curr_carry == self.zero_carry and abs(curr_value - self.zero_value) < 1500) or (curr_carry == 0 and curr_value < 10):
                        recovered = True
                        break
                except:
                    pass
                    
            if recovered:
                print(f"[{self.name}] \nRecovery successful! Motor returned to physical 0.")
                self.current_angle = 0.0
                self.current_pulses = 0
            else:
                print(f"\n[{self.name}] [FATAL] Recovery failed: Motor did not confirm return to 0 degrees.")
                print(f"[{self.name}] Angle tracking is NOT reset to protect hardware bounds. Resolve jam manually.")
        except Exception as rec_err:
            print(f"\n[{self.name}] [FATAL] Failed to execute 'Goto 0': {rec_err}. Angle tracking frozen.")


def main():
    print(f"Connecting to serial bus on {COM_PORT}...")
    try:
        bus = MksServoBus(port=COM_PORT, baudrate=BAUD_RATE, timeout=10.0, debug=False)
    except Exception as e:
        print(f"Critical Error: Failed to open port {COM_PORT}: {e}")
        sys.exit(1)

    # Initialize the axes objects
    tilt_axis = PanTiltAxis(
        name="TILT", 
        bus=bus, 
        address=0xE1,          # Tilt motor
        gear_ratio=4.0, 
        min_angle=-30.0, 
        max_angle=80.0
    )
    
    pan_axis = PanTiltAxis(
        name="PAN", 
        bus=bus, 
        address=0xE2,          # Pan motor
        gear_ratio=5.0, 
        min_angle=-180.0, 
        max_angle=180.0
    )

    try:
        tilt_axis.init_motor()
        time.sleep(0.1) # brief pause between motor inits
        pan_axis.init_motor()

        print("\n=========================================")
        print("      Pan-Tilt Dual Controller Ready     ")
        print("=========================================")
        print(f"TILT Axis: Addr=0xE1, Ratio=4:1, Limits=[-30°, 80°]")
        print(f"PAN Axis:  Addr=0xE0, Ratio=5:1, Limits=[-180°, 180°]")
        print("=========================================\n")
        
        while True:
            try:
                user_input = input("Enter command -> format: '[axis] [angle] [speed]' (e.g., 'tilt 45', 'pan 90 1.5', or 'q' to quit): ")
                
                parts = user_input.strip().split()
                if not parts:
                    continue
                    
                if parts[0].lower() == 'q':
                    break
                    
                if len(parts) < 2:
                    print("Error: Please provide at least an axis and an angle. (e.g. 'pan 90')")
                    continue
                    
                axis_name = parts[0].lower()
                try:
                    target_angle = float(parts[1])
                    speed_multiplier = float(parts[2]) if len(parts) > 2 else 1.0
                except ValueError:
                    print("Error: Invalid numeric input. Example: 'tilt -20' or 'pan 180 2.0'")
                    continue
                    
                if axis_name == 'tilt':
                    tilt_axis.move_to(target_angle, speed_multiplier)
                elif axis_name == 'pan':
                    pan_axis.move_to(target_angle, speed_multiplier)
                else:
                    print(f"Error: Unknown axis '{axis_name}'. Please use 'pan' or 'tilt'.")
                    
            except EOFError:
                break # Handle Ctrl+D smoothly

    except KeyboardInterrupt:
        print("\nInterrupted by user (Ctrl+C).")
    except Exception as e:
        print(f"\nUnexpected Critical Error: {e}")
    finally:
        print("\nInitiating cleanup sequence...")
        try:
            tilt_axis.motor.stop()
            tilt_axis.motor.set_enable(False)
        except: pass
        try:
            pan_axis.motor.stop()
            pan_axis.motor.set_enable(False)
        except: pass
        try:
            bus.close()
        except: pass
        print("Cleanup completed. System exit.")

if __name__ == "__main__":
    main()
