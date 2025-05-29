import serial
import time
import sys
from evdev import InputDevice, categorize, ecodes

def reflect8(byte):
    """Reflect the bits of an 8-bit value."""
    reflected = 0
    for i in range(8):
        if byte & (1 << i):
            reflected |= 1 << (7 - i)
    return reflected

def crc8_maxim(data):
    """
    Calculate CRC-8/MAXIM checksum for a 9-byte input.
    Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
    """
    if len(data) != 9:
        raise ValueError("Input data must be 9 bytes")
    
    crc = 0x00
    polynomial = 0x31
    
    for byte in data:
        byte = reflect8(byte)
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ polynomial) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    
    crc = reflect8(crc)
    crc ^= 0x00
    
    return list(data) + [crc]

def generate_speed_command(motor_id, rpm):
    """
    Generate a 9-byte command for the given motor ID and RPM.
    Format: [ID, 0x64, RPM_high, RPM_low, 0x00, 0x00, 0x00, 0x00, 0x00]
    """
    if motor_id not in [0x01, 0x02, 0x03, 0x04]:
        raise ValueError("Motor ID must be 0x01, 0x02, 0x03, or 0x04")
    if not -32768 <= rpm <= 32767:
        raise ValueError("RPM must be between -32768 and 32767")
    
    rpm = int(rpm) & 0xFFFF
    rpm_high = (rpm >> 8) & 0xFF
    rpm_low = rpm & 0xFF
    
    command = [motor_id, 0x64, rpm_high, rpm_low, 0x00, 0x00, 0x00, 0x00, 0x00]
    return command

def send_command(ser, command):
    """
    Send a 10-byte command to DDSM115 via RS485 and read response.
    """
    try:
        command_with_crc = crc8_maxim(command)
        command_bytes = bytes(command_with_crc)
        
        motor_id = command[0]
        rpm = (command[2] << 8) | command[3]
        if command[2] & 0x80:
            rpm -= 0x10000
        position = {0x01: "Right-Rear", 0x02: "Left-Front", 0x03: "Left-Rear", 0x04: "Right-Front"}
        timestamp = time.strftime("%H:%M:%S.%f")[:-3]
        print(f"[{timestamp}] Motor ID 0x{motor_id:02X} ({position[motor_id]}): Sending {' '.join(f'{b:02X}' for b in command_with_crc)} (RPM: {rpm})")
        ser.write(command_bytes)
        
        response = ser.read(10)
        if response:
            print(f"[{timestamp}] Motor ID 0x{motor_id:02X} ({position[motor_id]}): Received {' '.join(f'{b:02X}' for b in response)}")
        else:
            print(f"[{timestamp}] Motor ID 0x{motor_id:02X} ({position[motor_id]}): No response received")
        
        return response
    except serial.SerialException as e:
        print(f"[{timestamp}] Motor ID 0x{motor_id}: {e}")
        return None
    except ValueError as e:
        print(f"[{timestamp}] {e}")
        return None

def control_with_gamepad(port, gamepad_device):
    """
    Control the vehicle using ShanWan gamepad joysticks at 0–50 RPM (wheel diameter 0.64m).
    - Right Joystick (ABS_RZ): Forward/Backward, variable speed.
    - Left Joystick (ABS_X): Right/Left turns, variable speed.
    Moves only while joystick is tilted; stops on release.
    Key mappings:
    - Right Joystick Up (ABS_RZ < 127): Forward, 0–50 RPM
    - Right Joystick Down (ABS_RZ > 127): Backward, 0–50 RPM
    - Left Joystick Left (ABS_X < 127): Left turn, 0–50 RPM
    - Left Joystick Right (ABS_X > 127): Right turn, 0–50 RPM
    - Start: Quit
    """
    try:
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
    except serial.SerialException as e:
        print(f"[{time.strftime('%H:%M:%S.%f')[:-3]}] Failed to open serial port: {e}")
        return

    try:
        gamepad = InputDevice(gamepad_device)
        print(f"[{time.strftime('%H:%M:%S.%f')[:-3]}] Connected to gamepad: {gamepad.name}")
    except Exception as e:
        print(f"[{time.strftime('%H:%M:%S.%f')[:-3]}] Failed to open gamepad: {e}")
        ser.close()
        return

    motor_ids = [0x01, 0x02, 0x03, 0x04]
    max_rpm = 50  # Max variable speed
    dead_zone_min = 112  # Dead zone (128 ± 15)
    dead_zone_max = 142
    center = 128
    stop_command = {id: generate_speed_command(id, 0) for id in motor_ids}
    
    movements = {
        'forward': {0x02: 0, 0x04: 0, 0x03: 0, 0x01: 0},  # Variable Forward
        'backward': {0x02: 0, 0x04: 0, 0x03: 0, 0x01: 0},  # Variable Backward
        'left': {0x01: 0, 0x02: 0, 0x03: 0, 0x04: 0},  # Variable Left
        'right': {0x01: 0, 0x02: 0, 0x03: 0, 0x04: 0}  # Variable Right
    }
    
    current_movement = None
    last_command_time = 0
    command_interval = 0.15
    current_rpm = 0

    def send_movement(key_name, rpm):
        timestamp = time.strftime("%H:%M:%S.%f")[:-3]
        print(f"[{timestamp}] === Moving {key_name.capitalize()} (RPM: {rpm:.1f}) ===")
        if key_name == 'forward':
            movements[key_name] = {0x02: rpm, 0x04: -rpm, 0x03: rpm, 0x01: -rpm}
        elif key_name == 'backward':
            movements[key_name] = {0x02: -rpm, 0x04: rpm, 0x03: -rpm, 0x01: rpm}
        elif key_name == 'left':
            movements[key_name] = {0x01: -rpm, 0x02: -rpm, 0x03: -rpm, 0x04: -rpm}
        elif key_name == 'right':
            movements[key_name] = {0x01: rpm, 0x02: rpm, 0x03: rpm, 0x04: rpm}
        for motor_id in motor_ids:
            command = generate_speed_command(motor_id, movements[key_name][motor_id])
            send_command(ser, command)

    def send_stop():
        timestamp = time.strftime("%H:%M:%S.%f")[:-3]
        print(f"[{timestamp}] === Stopping Motors ===")
        ser.flushOutput()
        for motor_id in motor_ids:
            command = stop_command[motor_id]
            send_command(ser, command)

    try:
        for event in gamepad.read_loop():
            current_time = time.time()
            timestamp = time.strftime("%H:%M:%S.%f")[:-3]

            if event.type == ecodes.EV_KEY and event.value == 1:  # Button press
                if event.code == ecodes.BTN_START:  # Quit
                    print(f"[{timestamp}] Key: Start (Quit)")
                    send_stop()
                    break

            elif event.type == ecodes.EV_ABS:  # Joysticks
                if event.code == ecodes.ABS_RZ:  # Right Joystick Forward/Backward
                    if event.value < dead_zone_min:  # Forward
                        rpm = max_rpm * (dead_zone_min - event.value) / (dead_zone_min)
                        if current_movement != 'forward' or abs(current_rpm - rpm) > 1:
                            current_movement = 'forward'
                            current_rpm = rpm
                            send_movement('forward', rpm)
                            print(f"[{timestamp}] Key: Right Joystick Forward (RPM: {rpm:.1f})")
                    elif event.value > dead_zone_max:  # Backward
                        rpm = max_rpm * (event.value - dead_zone_max) / (255 - dead_zone_max)
                        if current_movement != 'backward' or abs(current_rpm - rpm) > 1:
                            current_movement = 'backward'
                            current_rpm = rpm
                            send_movement('backward', rpm)
                            print(f"[{timestamp}] Key: Right Joystick Backward (RPM: {rpm:.1f})")
                    elif dead_zone_min <= event.value <= dead_zone_max:  # Stop
                        if current_movement in ['forward', 'backward']:
                            current_movement = None
                            current_rpm = 0
                            send_stop()
                            print(f"[{timestamp}] Key: Right Joystick Released")

                elif event.code == ecodes.ABS_X:  # Left Joystick Left/Right
                    if event.value < dead_zone_min:  # Left
                        rpm = max_rpm * (dead_zone_min - event.value) / (dead_zone_min)
                        if current_movement != 'left' or abs(current_rpm - rpm) > 1:
                            current_movement = 'left'
                            current_rpm = rpm
                            send_movement('left', rpm)
                            print(f"[{timestamp}] Key: Left Joystick Left (RPM: {rpm:.1f})")
                    elif event.value > dead_zone_max:  # Right
                        rpm = max_rpm * (event.value - dead_zone_max) / (255 - dead_zone_max)
                        if current_movement != 'right' or abs(current_rpm - rpm) > 1:
                            current_movement = 'right'
                            current_rpm = rpm
                            send_movement('right', rpm)
                            print(f"[{timestamp}] Key: Left Joystick Right (RPM: {rpm:.1f})")
                    elif dead_zone_min <= event.value <= dead_zone_max:  # Stop
                        if current_movement in ['left', 'right']:
                            current_movement = None
                            current_rpm = 0
                            send_stop()
                            print(f"[{timestamp}] Key: Left Joystick Released")

            # Continue current movement if held
            if current_movement and current_time - last_command_time >= command_interval:
                send_movement(current_movement, current_rpm)
                last_command_time = current_time

    except KeyboardInterrupt:
        timestamp = time.strftime("%H:%M:%S.%f")[:-3]
        print(f"[{timestamp}] Interrupted! Sending stop commands...")
        send_stop()
        print("Exiting...")
    finally:
        send_stop()
        ser.close()

def main(port, gamepad_device):
    try:
        control_with_gamepad(port, gamepad_device)
        print("\nProgram terminated.")
    except Exception as e:
        timestamp = time.strftime("%H:%M:%S.%f")[:-3]
        print(f"[{timestamp}] Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 wifirc2.py <serial_port> <gamepad_device>")
        print("Example: python3 wifirc2.py /dev/ttyACM0 /dev/input/event14")
        sys.exit(1)
    
    port = sys.argv[1]
    gamepad_device = sys.argv[2]
    main(port, gamepad_device)



######################################################################################################################################################################
# To run:
# lsusb
# dmesg | grep -i shanwan
# sudo dmesg | grep -i shanwan
# ls /dev/input/
# sudo apt update
# sudo apt install evtest
# sudo evtest
# sudo apt update
# sudo apt install xboxdrv
# sudo xboxdrv --detach-kernel-driver --device-by-id 2563:0526
# ls -l /dev/ttyACM0
# ls -l /dev/input/event14
# sudo chmod 666 /dev/input/event14
# python3 wifirc.py /dev/ttyACM0 /dev/input/event14
