import serial
import time
import sys

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
    Initial value: 0x00
    RefIn: True (input bytes are reflected)
    RefOut: True (output CRC is reflected)
    XorOut: 0x00
    """
    if len(data) != 9:
        raise ValueError("Input data must be exactly 9 bytes")
    
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
    if not -32768 <= rpm <= 32767:  # 16-bit signed RPM
        raise ValueError("RPM must be between -32768 and 32767")
    
    # Convert RPM to 16-bit big-endian (signed)
    rpm = rpm & 0xFFFF  # Handle negative RPM (two's complement)
    rpm_high = (rpm >> 8) & 0xFF
    rpm_low = rpm & 0xFF
    
    # Base command
    command = [motor_id, 0x64, rpm_high, rpm_low, 0x00, 0x00, 0x00, 0x00, 0x00]
    return command

def send_command(port, command, timeout=1):
    """
    Send a 10-byte command to DDSM115 via RS485 and read response.
    Returns response bytes or None if no response.
    """
    try:
        # Calculate CRC
        command_with_crc = crc8_maxim(command)
        command_bytes = bytes(command_with_crc)
        
        # Open serial port
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        
        # Send command
        motor_id = command[0]
        rpm = (command[2] << 8) | command[3]
        if command[2] & 0x80:  # Handle negative RPM
            rpm -= 0x10000
        print(f"Motor ID 0x{motor_id:02X}: Sending {' '.join(f'{b:02X}' for b in command_with_crc)} (RPM: {rpm})")
        ser.write(command_bytes)
        
        # Read response (up to 10 bytes)
        response = ser.read(10)
        if response:
            print(f"Motor ID 0x{motor_id:02X}: Received {' '.join(f'{b:02X}' for b in response)}")
        else:
            print(f"Motor ID 0x{motor_id:02X}: No response received")
        
        # Close port
        ser.close()
        
        return response if response else None
    except serial.SerialException as e:
        print(f"Motor ID 0x{motor_id:02X}: Serial error: {e}")
        return None
    except ValueError as e:
        print(f"Motor ID 0x{motor_id:02X}: Error: {e}")
        return None

def control_motors(port):
    """
    Control four DDSM115 motors (IDs 0x01, 0x02, 0x03, 0x04) to move:
    - Straight (all +50 RPM) for 5s
    - Backward (all -50 RPM) for 5s
    - Left (01,03: -50 RPM; 02,04: +50 RPM) for 5s
    - Right (01,03: +50 RPM; 02,04: -50 RPM) for 5s
    """
    motor_ids = [0x01, 0x02, 0x03, 0x04]
    speed_rpm = 50  # Base speed
    stop_command = {id: generate_speed_command(id, 0) for id in motor_ids}
    
    movements = [
        ("Straight", {0x01: speed_rpm, 0x02: speed_rpm, 0x03: speed_rpm, 0x04: speed_rpm}),
        ("Backward", {0x01: -speed_rpm, 0x02: -speed_rpm, 0x03: -speed_rpm, 0x04: -speed_rpm}),
        ("Turn Left", {0x01: -speed_rpm, 0x02: speed_rpm, 0x03: -speed_rpm, 0x04: speed_rpm}),
        ("Turn Right", {0x01: speed_rpm, 0x02: -speed_rpm, 0x03: speed_rpm, 0x04: -speed_rpm})
    ]
    
    try:
        for movement_name, rpm_settings in movements:
            print(f"\n=== Starting {movement_name} ===")
            # Send commands to all motors
            for motor_id in motor_ids:
                command = generate_speed_command(motor_id, rpm_settings[motor_id])
                send_command(port, command)
            
            # Wait 5 seconds
            time.sleep(5)
            
            # Stop all motors
            print(f"\nStopping motors before next movement...")
            for motor_id in motor_ids:
                send_command(port, stop_command[motor_id])
            
            # Wait 1 second to settle
            time.sleep(1)
        
        # Final stop
        print(f"\n=== Final Stop ===")
        for motor_id in motor_ids:
            send_command(port, stop_command[motor_id])
        
        print("\nMovement sequence completed.")
    
    except KeyboardInterrupt:
        print("\nInterrupted! Sending stop commands...")
        for motor_id in motor_ids:
            send_command(port, stop_command[motor_id])
        print("Exiting...")
        sys.exit(0)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 control_four_ddsm115.py <port>")
        print("Example: python3 control_four_ddsm115.py /dev/ttyUSB0")
        sys.exit(1)
    
    port = sys.argv[1]
    
    print("Starting movement sequence. Ensure motors are secured!")
    control_motors(port)
