import serial
import time
import sys
import curses

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
    if not -32768 <= rpm <= 32767:
        raise ValueError("RPM must be between -32768 and 32767")
    
    rpm = rpm & 0xFFFF
    rpm_high = (rpm >> 8) & 0xFF
    rpm_low = rpm & 0xFF
    
    command = [motor_id, 0x64, rpm_high, rpm_low, 0x00, 0x00, 0x00, 0x00, 0x00]
    return command

def send_command(ser, command):
    """
    Send a 10-byte command to DDSM115 via RS485 and read response.
    Returns response bytes or None.
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
        print(f"[{timestamp}] Motor ID 0x{motor_id:02X}: {e}")
        return None
    except ValueError as e:
        print(f"[{timestamp}] Error: {e}")
        return None

def control_with_arrow_keys(stdscr, port):
    """
    Control the vehicle using single arrow key presses at 15 RPM (wheel diameter 0.64m).
    Motor placement and facing:
    - 2 (0x02, Left-Front): Faces left, CW (+15 RPM) for forward, CCW (-15 RPM) for backward
    - 4 (0x04, Right-Front): Faces right, CCW (-15 RPM) for forward, CW (+15 RPM) for backward
    - 3 (0x03, Left-Rear): Faces left, CW (+15 RPM) for forward, CCW (-15 RPM) for backward
    - 1 (0x01, Right-Rear): Faces right, CCW (-15 RPM) for forward, CW (+15 RPM) for backward
    Key mappings:
    - Up: Start Forward (2,3 CW, 4,1 CCW)
    - Down: Start Backward (2,3 CCW, 4,1 CW)
    - Left: Start Turn Left (all CCW)
    - Right: Start Turn Right (all CW)
    - Space: Stop
    - q: Quit
    """
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)
    stdscr.clear()
    stdscr.addstr(0, 0, "Press arrow keys to start movement, space to stop, 'q' to quit")
    stdscr.refresh()

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
        stdscr.addstr(2, 0, f"Failed to open serial port: {e}")
        stdscr.refresh()
        time.sleep(2)
        return

    motor_ids = [0x01, 0x02, 0x03, 0x04]
    speed_rpm = 50
    stop_command = {id: generate_speed_command(id, 0) for id in motor_ids}
    
    movements = {
        'up': {0x02: speed_rpm, 0x04: -speed_rpm, 0x03: speed_rpm, 0x01: -speed_rpm},  # Forward
        'down': {0x02: -speed_rpm, 0x04: speed_rpm, 0x03: -speed_rpm, 0x01: speed_rpm},  # Backward
        'left': {0x01: -speed_rpm, 0x02: -speed_rpm, 0x03: -speed_rpm, 0x04: -speed_rpm},  # Left
        'right': {0x01: speed_rpm, 0x02: speed_rpm, 0x03: speed_rpm, 0x04: speed_rpm}  # Right
    }
    
    current_movement = None
    last_command_time = 0
    command_interval = 0.15  # 150ms

    def send_movement(key_name):
        timestamp = time.strftime("%H:%M:%S.%f")[:-3]
        stdscr.addstr(2, 0, f"[{timestamp}] Moving: {key_name.capitalize()}        ")
        stdscr.refresh()
        print(f"[{timestamp}] === Moving {key_name.capitalize()} ===")
        for motor_id in motor_ids:
            command = generate_speed_command(motor_id, movements[key_name][motor_id])
            send_command(ser, command)

    def send_stop():
        timestamp = time.strftime("%H:%M:%S.%f")[:-3]
        stdscr.addstr(2, 0, f"[{timestamp}] Stopped                ")
        stdscr.refresh()
        print(f"[{timestamp}] === Stopping Motors ===")
        ser.flushOutput()
        for motor_id in motor_ids:
            command = stop_command[motor_id]
            send_command(ser, command)

    try:
        while True:
            try:
                key = stdscr.getch()
                current_time = time.time()

                timestamp = time.strftime("%H:%M:%S.%f")[:-3]
                if key == ord('q'):
                    stdscr.addstr(3, 0, f"[{timestamp}] Key: Quit        ")
                    send_stop()
                    break
                elif key == ord(' '):
                    if current_movement:
                        current_movement = None
                        send_stop()
                    stdscr.addstr(3, 0, f"[{timestamp}] Key: Space       ")
                elif key == curses.KEY_UP and current_movement != 'up':
                    current_movement = 'up'
                    send_movement('up')
                    last_command_time = current_time
                    stdscr.addstr(3, 0, f"[{timestamp}] Key: Up          ")
                elif key == curses.KEY_DOWN and current_movement != 'down':
                    current_movement = 'down'
                    send_movement('down')
                    last_command_time = current_time
                    stdscr.addstr(3, 0, f"[{timestamp}] Key: Down        ")
                elif key == curses.KEY_LEFT and current_movement != 'left':
                    current_movement = 'left'
                    send_movement('left')
                    last_command_time = current_time
                    stdscr.addstr(3, 0, f"[{timestamp}] Key: Left        ")
                elif key == curses.KEY_RIGHT and current_movement != 'right':
                    current_movement = 'right'
                    send_movement('right')
                    last_command_time = current_time
                    stdscr.addstr(3, 0, f"[{timestamp}] Key: Right       ")

                # Continue current movement
                if current_movement and current_time - last_command_time >= command_interval:
                    send_movement(current_movement)
                    last_command_time = current_time

                stdscr.addstr(1, 0, f"Last key code: {key}        ")
                stdscr.refresh()
                curses.napms(10)

            except curses.error:
                pass

    except KeyboardInterrupt:
        timestamp = time.strftime("%H:%M:%S.%f")[:-3]
        print(f"[{timestamp}] Interrupted! Sending stop commands...")
        send_stop()
        print("Exiting...")
    finally:
        send_stop()
        ser.close()

def main(port):
    try:
        curses.wrapper(lambda stdscr: control_with_arrow_keys(stdscr, port))
        print("\nProgram terminated.")
    except Exception as e:
        timestamp = time.strftime("%H:%M:%S.%f")[:-3]
        print(f"[{timestamp}] Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 arrowcontrol.py <port>")
        print("Example: python3 arrowcontrol.py /dev/ttyACM0")
        sys.exit(1)
    
    port = sys.argv[1]
    main(port)
