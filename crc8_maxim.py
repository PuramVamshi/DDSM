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
    
    Args:
        data: A list or bytes-like object of 9 bytes (DATA[0]~DATA[8])
        
    Returns:
        A list of 10 bytes with the CRC as the last byte
    """
    if len(data) != 9:
        raise ValueError("Input data must be exactly 9 bytes")
    
    crc = 0x00
    polynomial = 0x31  # x^8 + x^5 + x^4 + 1
    
    # Process bytes in MSB-first order
    for byte in data:
        # Reflect input byte (RefIn: True)
        byte = reflect8(byte)
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ polynomial) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    
    # Reflect output (RefOut: True)
    crc = reflect8(crc)
    # Apply final XOR (XorOut: 0x00)
    crc ^= 0x00
    
    return list(data) + [crc]

def hex_to_bytes(hex_str):
    """Convert a hex string (e.g., '01A000000000000000' or '01 A0 00 00 00 00 00 00 00') to a list of 9 bytes."""
    # Remove spaces and normalize
    hex_str = ''.join(hex_str.split())
    if len(hex_str) != 18:
        raise ValueError("Hex string must represent exactly 9 bytes (18 hex chars)")
    try:
        bytes_list = [int(hex_str[i:i+2], 16) for i in range(0, 18, 2)]
        return bytes_list
    except ValueError:
        raise ValueError("Invalid hex string")

# Interactive hex string input
while True:
    try:
        hex_input = input("Enter 9-byte hex input (e.g., '01A000000000000000' or '01 A0 00 00 00 00 00 00 00', or 'q' to quit): ").strip()
        if hex_input.lower() == 'q':
            break
        data = hex_to_bytes(hex_input)
        result = crc8_maxim(data)
        print(f"Input:  {' '.join(f'{b:02X}' for b in data)}")
        print(f"Output: {' '.join(f'{b:02X}' for b in result)}")
        print(f"CRC: {result[-1]:02X}")
        print()
    except ValueError as e:
        print(f"Error: {e}")
        print()
    except KeyboardInterrupt:
        print("\nExiting...")
        break
