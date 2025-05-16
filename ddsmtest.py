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

def generate_speed_command(rpm):
    """
    Generate a 9-byte command for the given RPM (e.g., 50 RPM = 0x0032).
    Format: [0x01, 0x64, RPM_high, RPM_low, 0x00, 0x00, 0x00, 0x00, 0x00]
    """
    if not 0 <= rpm <= 65535:  # 16-bit RPM value
        raise ValueError("RPM must be between 0 and 65535")
    
    # Convert RPM to 16-bit big-endian
    rpm_high = (rpm >> 8) & 0xFF
    rpm_low = rpm & 0xFF
    
    # Base command
    command = [0x01, 0x64, rpm_high, rpm_low, 0x00, 0x00, 0x00, 0x00, 0x00]
    return command

def send_command(port, command):
    """
    Send a 10-byte command (9 bytes + CRC) to DDSM115 via RS485 and read response.
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
            timeout=1
        )
        
        # Send command
        print(f"Sending: {' '.join(f'{b:02X}' for b in command_with_crc)} (RPM: {command[2]*256 + command[3]})")
        ser.write(command_bytes)
        
        # Read response (up to 10 bytes)
        response = ser.read(10)
        if response:
            print(f"Received: {' '.join(f'{b:02X}' for b in response)}")
        else:
            print("No response received")
        
        # Close port
        ser.close()
        
        return True
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return False
    except ValueError as e:
        print(f"Error: {e}")
        return False

def run_speed_sequence(port):
    """
    Run DDSM115 at 10, 20, ..., 200 RPM, each for 10 seconds, with 5-second stops.
    """
    # Stop command (0 RPM)
    stop_command = generate_speed_command(0)
    
    # Speed sequence: 10 to 200 RPM in steps of 10
    for rpm in range(10, 201, 10):
        print(f"\nRunning at {rpm} RPM...")
        # Generate and send speed command
        speed_command = generate_speed_command(rpm)
        if not send_command(port, speed_command):
            print("Failed to send speed command. Aborting.")
            return
        
        # Run for 10 seconds
        time.sleep(10)
        
        # Send stop command
        print("\nStopping motor (0 RPM)...")
        if not send_command(port, stop_command):
            print("Failed to send stop command. Aborting.")
            return
        
        # Wait 5 seconds
        time.sleep(5)
    
    # Final stop
    print("\nFinal stop after 200 RPM...")
    send_command(port, stop_command)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 run_ddsm115_speeds.py <port>")
        print("Example: python3 run_ddsm115_speeds.py /dev/ttyUSB0")
        sys.exit(1)
    
    port = sys.argv[1]
    
    try:
        print("Starting speed sequence. Ensure motor is secured!")
        run_speed_sequence(port)
        print("Sequence completed.")
    except KeyboardInterrupt:
        print("\nInterrupted! Sending stop command...")
        stop_command = generate_speed_command(0)
        send_command(port, stop_command)
        print("Exiting...")
        sys.exit(0)
