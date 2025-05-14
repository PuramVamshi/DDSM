import serial
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

def hex_to_bytes(hex_str):
    """Convert a hex string (e.g., '01A000000000000000' or '01 A0 00 00 00 00 00 00 00') to a list of 9 bytes."""
    hex_str = ''.join(hex_str.split())
    if len(hex_str) != 18:
        raise ValueError("Hex string must represent exactly 9 bytes (18 hex chars)")
    try:
        return [int(hex_str[i:i+2], 16) for i in range(0, 18, 2)]
    except ValueError:
        raise ValueError("Invalid hex string")

def send_command(port, hex_input):
    """Send a 10-byte command (9 bytes + CRC) to DDSM115 via RS485 and read response."""
    try:
        # Convert hex input to bytes
        data = hex_to_bytes(hex_input)
        # Calculate CRC
        command = crc8_maxim(data)
        # Convert to bytes
        command_bytes = bytes(command)
        
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
        print(f"Sending: {' '.join(f'{b:02X}' for b in command)}")
        ser.write(command_bytes)
        
        # Read response (if any, e.g., 10 bytes)
        response = ser.read(10)
        if response:
            print(f"Received: {' '.join(f'{b:02X}' for b in response)}")
        else:
            print("No response received")
        
        # Close port
        ser.close()
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except ValueError as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 send_ddsm115_command.py <port> <9-byte-hex>")
        print("Example: python3 send_ddsm115_command.py /dev/ttyUSB0 '01 64 F8 30 00 00 00 00 00'")
        sys.exit(1)
    
    port = sys.argv[1]
    hex_input = sys.argv[2]
    
    try:
        send_command(port, hex_input)
    except KeyboardInterrupt:
        print("\nExiting...")
        sys.exit(0)
