"""
Servo Control Script for ESP32-S3 Dual Channel PWM Control

This script sends pulsewidth commands to the ESP32-S3 to control two servo channels:
- Channel 0 (Azimuth): 500-2500 μs range
- Channel 1 (Elevation): 800-1500 μs range

The servos sweep back and forth continuously at an adjustable rate.

Requirements:
    pip install pyserial
"""

import sys
import time
import argparse

try:
    import serial
except ImportError:
    print("Error: pyserial module not found!")
    print("\nPlease install it using:")
    print("    pip install pyserial")
    print("\nOr with conda:")
    print("    conda install pyserial")
    sys.exit(1)

class ServoController:
    def __init__(self, port='COM3', baudrate=115200, update_rate_hz=50):
        """
        Initialize servo controller
        
        Args:
            port: Serial port name (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux)
            baudrate: Serial communication baudrate
            update_rate_hz: Update rate in Hz (max 50Hz recommended)
        """
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.update_period = 1.0 / update_rate_hz
        
        # Channel 0 (Azimuth) configuration
        self.ch0_min = 500
        self.ch0_max = 2500
        self.ch0_current = (self.ch0_min + self.ch0_max) // 2
        self.ch0_step = 10
        
        # Channel 1 (Elevation) configuration
        self.ch1_min = 800
        self.ch1_max = 1500
        self.ch1_current = (self.ch1_min + self.ch1_max) // 2
        self.ch1_step = 5
        
        print(f"Connected to {port} at {baudrate} baud")
        print(f"Update rate: {update_rate_hz} Hz ({self.update_period*1000:.2f} ms period)")
        print(f"CH0 (Azimuth): {self.ch0_min}-{self.ch0_max} μs, step: {self.ch0_step}")
        print(f"CH1 (Elevation): {self.ch1_min}-{self.ch1_max} μs, step: {self.ch1_step}")
        print("\nPress Ctrl+C to stop\n")
        
    def update_positions(self):
        """Update servo positions with back-and-forth sweep"""
        # Update CH0 (Azimuth)
        self.ch0_current += self.ch0_step
        if self.ch0_current >= self.ch0_max or self.ch0_current <= self.ch0_min:
            self.ch0_step = -self.ch0_step
            self.ch0_current += self.ch0_step  # Correct overshoot
        
        # Update CH1 (Elevation)
        self.ch1_current += self.ch1_step
        if self.ch1_current >= self.ch1_max or self.ch1_current <= self.ch1_min:
            self.ch1_step = -self.ch1_step
            self.ch1_current += self.ch1_step  # Correct overshoot
    
    def send_command(self):
        """Send current positions to ESP32 and wait for acknowledgment"""
        command = f"{self.ch0_current} {self.ch1_current}\n"
        self.ser.write(command.encode())
        
        # Wait for ACK from ESP32 (with timeout)
        start_wait = time.time()
        while time.time() - start_wait < 0.1:  # 100ms timeout
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode('utf-8', errors='ignore').strip()
                # Look for ACK in ESP-IDF log format: "I (timestamp) SerialThread: ACK"
                if "SerialThread:" in response and "ACK" in response:
                    return True
        return False  # Timeout - no ACK received
    
    def read_esp_feedback(self):
        """Read and parse ESP32 position feedback"""
        try:
            # Read available lines from serial
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # Look for ServoThread log messages with format: "I (timestamp) ServoThread: CH0: 1500 us, CH1: 1150 us"
                    if "ServoThread:" in line and "CH0:" in line and "CH1:" in line:
                        try:
                            # Extract CH0 and CH1 values
                            parts = line.split("ServoThread:")[1].strip()
                            ch0_part = parts.split("CH0:")[1].split("us")[0].strip()
                            ch1_part = parts.split("CH1:")[1].split("us")[0].strip()
                            
                            esp_ch0 = int(ch0_part)
                            esp_ch1 = int(ch1_part)
                            
                            return esp_ch0, esp_ch1
                        except (ValueError, IndexError):
                            pass
        except Exception as e:
            pass
        
        return None, None
    
    def run(self):
        """Main control loop"""
        try:
            cycle_count = 0
            last_esp_ch0 = None
            last_esp_ch1 = None
            
            while True:
                start_time = time.time()
                
                # Update positions
                self.update_positions()
                
                # Send to ESP32 and wait for ACK
                ack_received = self.send_command()
                
                # Read ESP32 feedback
                esp_ch0, esp_ch1 = self.read_esp_feedback()
                if esp_ch0 is not None:
                    last_esp_ch0 = esp_ch0
                    last_esp_ch1 = esp_ch1
                
                # Display: sent vs actual
                ack_status = "✓" if ack_received else "✗"
                if last_esp_ch0 is not None:
                    diff_ch0 = abs(self.ch0_current - last_esp_ch0)
                    diff_ch1 = abs(self.ch1_current - last_esp_ch1)
                    match_status = "✓" if (diff_ch0 <= 5 and diff_ch1 <= 5) else "⚠"
                    print(f"{ack_status} Sent: CH0={self.ch0_current:4d} CH1={self.ch1_current:4d} | "
                          f"ESP: CH0={last_esp_ch0:4d} CH1={last_esp_ch1:4d} | "
                          f"Δ: CH0={diff_ch0:3d} CH1={diff_ch1:3d} {match_status}")
                else:
                    print(f"{ack_status} Sent: CH0={self.ch0_current:4d} μs | CH1={self.ch1_current:4d} μs (waiting for ESP feedback...)")
                
                cycle_count += 1
                
                # Maintain constant update rate
                elapsed = time.time() - start_time
                sleep_time = self.update_period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\n\nStopping servo control...")
            # Center both servos before exit
            center_ch0 = (self.ch0_min + self.ch0_max) // 2
            center_ch1 = (self.ch1_min + self.ch1_max) // 2
            self.ser.write(f"{center_ch0} {center_ch1}\n".encode())
            time.sleep(0.1)
            self.ser.close()
            print("Servos centered and serial port closed")

def main():
    parser = argparse.ArgumentParser(description='Control ESP32-S3 dual-channel servo system')
    parser.add_argument('-p', '--port', default='COM3', help='Serial port (default: COM3)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200, help='Baudrate (default: 115200)')
    parser.add_argument('-r', '--rate', type=int, default=50, help='Update rate in Hz (default: 50)')
    parser.add_argument('--ch0-step', type=int, default=10, help='CH0 step size (default: 10)')
    parser.add_argument('--ch1-step', type=int, default=5, help='CH1 step size (default: 5)')
    
    args = parser.parse_args()
    
    controller = ServoController(
        port=args.port,
        baudrate=args.baudrate,
        update_rate_hz=args.rate
    )
    
    # Apply custom step sizes if provided
    if args.ch0_step != 10:
        controller.ch0_step = args.ch0_step
        print(f"CH0 step size set to: {args.ch0_step}")
    if args.ch1_step != 5:
        controller.ch1_step = args.ch1_step
        print(f"CH1 step size set to: {args.ch1_step}")
    
    controller.run()

if __name__ == "__main__":
    main()
