import sys
import time
from driver import MksServoBus

# --- Configuration Constants ---
COM_PORT = "COM14"
BAUD_RATE = 38400
MOTOR_ADDR = 0xE0

# Motor and Mechanics Setup
MSTEP = 16
STEPS_PER_REV = 200
GEAR_RATIO_TILT = 4.0   # 64-tooth gear / 16-tooth pinion
MIN_TILT_ANGLE = -30.0  # Degrees
MAX_TILT_ANGLE = 80.0   # Degrees

# Calculations
PULSES_PER_MOTOR_REV = MSTEP * STEPS_PER_REV
PULSES_PER_SHAFT_REV = int(PULSES_PER_MOTOR_REV * GEAR_RATIO_TILT)

def main():
    print(f"Connecting to motor on {COM_PORT}...")
    try:
        # Increase timeout to allow for longer blocking motions without timing out
        bus = MksServoBus(port=COM_PORT, baudrate=BAUD_RATE, timeout=10.0, debug=False)
    except Exception as e:
        print(f"Critical Error: Failed to open port {COM_PORT}: {e}")
        sys.exit(1)

    motor = bus.motor(MOTOR_ADDR)

    try:
        print("Configuring motor parameters...")
        motor.set_mode_uart()
        motor.set_enable(True)       # Enable motor to lock position
        motor.set_mstep(MSTEP)
        motor.zero_set_current_position() # Ensure internal motor position starts at 0
        
        # Record the hardware encoder value at this confirmed zero position
        try:
            zero_carry, zero_value = motor.read_encoder()
            print(f"Recorded absolute zero encoder value: carry={zero_carry}, value={zero_value}")
        except Exception as e:
            print(f"Warning: Could not read initial encoder value: {e}")
            zero_carry, zero_value = 0, 0

        print("\n--- Pan-Tilt Motor Controller Ready ---")
        print(f"Gear Ratio:      {GEAR_RATIO_TILT}:1")
        print(f"Valid Range:     {MIN_TILT_ANGLE}° to {MAX_TILT_ANGLE}°")
        print("---------------------------------------")
        
        current_pulses = 0
        current_angle = 0.0

        while True:
            try:
                user_input = input(f"\nEnter tilt angle [{MIN_TILT_ANGLE}° to {MAX_TILT_ANGLE}°] [optional: speed multiplier] (or 'q' to quit): ")
                
                parts = user_input.strip().split()
                if not parts:
                    continue
                    
                if parts[0].lower() == 'q':
                    break
                    
                try:
                    target_angle = float(parts[0])
                    speed_multiplier = float(parts[1]) if len(parts) > 1 else 1.0
                except ValueError:
                    print("Error: Invalid input. Please enter valid numeric values (e.g., '45' or '45 1.5').")
                    continue
                    
                # Software Limit Check (Robustness)
                if not (MIN_TILT_ANGLE <= target_angle <= MAX_TILT_ANGLE):
                    print(f"Error: Target angle {target_angle}° is out of the locked physical bounds [{MIN_TILT_ANGLE}°, {MAX_TILT_ANGLE}°].")
                    continue
                    
                # Calculate required pulses on the motor for the given tilt angle
                target_pulses = int(round((target_angle / 360.0) * PULSES_PER_SHAFT_REV))
                delta_pulses = target_pulses - current_pulses
                
                if delta_pulses == 0:
                    print(f"Already at {target_angle}°.")
                    continue
                    
                direction_cw = delta_pulses >= 0
                pulses_to_move = abs(delta_pulses)
                # --- Dynamic Speed Calculation ---
                # We want larger angles to travel faster to maintain a similar total travel time.
                # Calculate the motor revolutions needed for this move
                abs_delta_angle = abs(target_angle - current_angle)
                motor_revs_needed = (abs_delta_angle * GEAR_RATIO_TILT) / 360.0
                
                # To achieve a snappier constant travel time, calculate the required RPM
                TARGET_TRAVEL_TIME = 0.25  # seconds
                desired_rpm = (motor_revs_needed / TARGET_TRAVEL_TIME) * 60.0
                
                # Clamp the base speed to prevent small moves from being too slow, and large moves from being dangerously fast
                MIN_MOTOR_RPM = 90.0
                MAX_MOTOR_RPM = 400.0
                base_rpm = max(MIN_MOTOR_RPM, min(desired_rpm, MAX_MOTOR_RPM))
                
                # Apply user-defined speed multiplier
                motor_rpm = base_rpm * speed_multiplier
                
                # Absolute physical hardware limits to prevent gear/motor damage regardless of multiplier
                motor_rpm = max(10.0, min(motor_rpm, 800.0))
                
                print(f"Moving to {target_angle}°...")
                print(f" -> Shaft Delta: {target_angle - current_angle}°")
                print(f" -> Motor Pulses: {pulses_to_move}, CW={direction_cw}")
                print(f" -> Dynamic Speed: {motor_rpm:.1f} RPM (Base: {base_rpm:.1f} RPM x {speed_multiplier})")
                
                try:
                    motor.run_pulses(rpm=motor_rpm, pulses=pulses_to_move, mstep=MSTEP, direction_cw=direction_cw)
                    current_pulses = target_pulses
                    current_angle = target_angle
                    print(f"Motion complete. Current position securely held at {current_angle}°.")
                except Exception as e:
                    print(f"\n[CRITICAL ERROR] Error during execution of motion: {e}")
                    print("Initiating emergency recovery: Sending 'Goto 0' command...")
                    
                    try:
                        bus._ser.reset_input_buffer()  # Clear stale or corrupted serial data
                        motor.zero_goto()
                        print("Waiting for motor to return to zero...")
                        
                        recovered = False
                        for i in range(10):
                            time.sleep(1.0)
                            try:
                                curr_carry, curr_value = motor.read_encoder()
                                print(f" -> Recovery encoder check: carry={curr_carry}, value={curr_value}")
                                
                                # Check if encoder reached original zero (with tolerance) or hard reset to 0
                                if (curr_carry == zero_carry and abs(curr_value - zero_value) < 1500) or (curr_carry == 0 and curr_value < 10):
                                    recovered = True
                                    break
                            except Exception as enc_err:
                                print(f" -> Could not read encoder during recovery: {enc_err}")
                                
                        if recovered:
                            print("\nRecovery successful! Motor returned to physical 0.")
                            current_angle = 0.0
                            current_pulses = 0
                        else:
                            print("\n[FATAL] Recovery failed: Motor did not confirm return to 0 degrees.")
                            print("To protect hardware bounds, angle tracking is NOT reset.")
                            print("Please resolve hardware jam and manually reposition to 0 before restarting.")
                            
                    except Exception as rec_err:
                        print(f"\n[FATAL] Failed to execute 'Goto 0': {rec_err}")
                        print("To protect hardware bounds, angle tracking is NOT reset.")
                        
                # Optionally check hardware encoder to verify
                try:
                    carry, value = motor.read_encoder()
                    print(f"[Debug] Hardware Encoder: carry={carry}, value={value}")
                except Exception as enc_e:
                    print(f"Warning: Could not read telemetry encoder: {enc_e}")
                    
            except EOFError:
                break # Handle Ctrl+D smoothly

    except KeyboardInterrupt:
        print("\nInterrupted by user (Ctrl+C).")
    except Exception as e:
        print(f"\nUnexpected Critical Error: {e}")
    finally:
        print("\nInitiating cleanup sequence...")
        try:
            motor.stop()
            # Disable the motor to release torque at script end (remove or comment out if hold torque is needed when idle)
            motor.set_enable(False) 
        except Exception as e:
            print(f"Error while stopping motor: {e}")
        try:
            bus.close()
        except Exception:
            pass
        print("Cleanup completed. System exit.")

if __name__ == "__main__":
    main()
