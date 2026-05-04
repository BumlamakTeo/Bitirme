# Dual-Core Servo Control with Serial Input

## Architecture

This implementation uses a dual-core approach to prevent serial communication jitter from affecting PWM timing:

- **Core 0**: Serial input thread (non-blocking, lower priority)
- **Core 1**: Servo control thread (50Hz precise timing, higher priority)

## Features

- **Jitter-free PWM output**: Servo control runs on dedicated Core 1
- **Non-blocking serial input**: Runs on Core 0, doesn't interrupt PWM
- **Real-time updates**: Up to 50Hz update rate from serial input
- **Dual channel control**: Independent control of GPIO 40 and GPIO 42

## Channel Configuration

- **Channel 0 (GPIO 40)**: 500-2500 μs (full servo range)
- **Channel 1 (GPIO 42)**: 800-1500 μs (0-60° range)

## Serial Input Format

The system accepts multiple input formats via USB serial:

### Format 1: Named channels
```
CH0:1500 CH1:1000
```

### Format 2: Space-separated values
```
1500 1000
```

### Format 3: Single channel update
```
CH0:1500
```
or
```
CH1:1000
```

## Usage Examples

1. **Center both servos**:
   ```
   1500 1150
   ```

2. **Set CH0 to minimum, CH1 to maximum**:
   ```
   CH0:500 CH1:1500
   ```

3. **Update only CH0**:
   ```
   CH0:2000
   ```

## Python Integration

The system is designed to work with Python scripts sending commands at up to 50Hz.

### Quick Start Script

A complete Python script (`servo_control.py`) is provided that sweeps both channels:

```bash
# Basic usage (50Hz update rate)
python servo_control.py

# Custom serial port
python servo_control.py -p COM5

# Adjust update rate (e.g., 20Hz)
python servo_control.py -r 20

# Custom step sizes for smoother/faster motion
python servo_control.py --ch0-step 5 --ch1-step 2

# Linux example
python servo_control.py -p /dev/ttyUSB0 -r 30
```

**Features:**
- CH0 sweeps 500-2500 μs (azimuth)
- CH1 sweeps 800-1500 μs (elevation)
- Adjustable update rate (1-50 Hz)
- Graceful shutdown (centers servos on Ctrl+C)
- Real-time status display

### Simple Example

```python
import serial
import time

ser = serial.Serial('COM26', 115200)  # Adjust port as needed

while True:
    ch0_pw = 1500  # Calculate desired pulsewidth
    ch1_pw = 1000  # Calculate desired pulsewidth
    
    ser.write(f"{ch0_pw} {ch1_pw}\n".encode())
    time.sleep(0.02)  # 50Hz update rate
```

## Build and Flash

```bash
idf.py build
idf.py flash monitor
```

## Monitoring

Watch the serial output to see:
- Thread initialization on each core
- Real-time pulsewidth updates
- Parsed commands and validation

## Thread Safety

- Pulsewidth updates are atomic (simple integer assignment)
- No mutex needed due to single-writer (serial thread), single-reader (servo thread) pattern
- Range clamping ensures values stay within valid bounds
