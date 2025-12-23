import serial
import struct

usb_port = 'COM6'  # Update this to your port
baud_rate = 115200

ser = serial.Serial(usb_port, baud_rate, timeout=1)

def read_packet():
    while True:
        byte = ser.read(1)
        if not byte:
            continue

        if byte[0] == 0xAA:  # Found header
            # Read remaining 10 bytes of the packet
            data = ser.read(10)
            if len(data) < 10:
                continue  # incomplete packet, discard and resync
            
            # data layout:
            # byte 0: greenPressed (uint8)
            # byte 1: bluePressed (uint8)
            # byte 2: extKillState (uint8)
            # byte 3: intKillState (uint8)
            # byte 4-7: depth (float, little-endian)
            # byte 8: '\r' (ignore)
            # byte 9: '\n' (ignore)

            greenPressed = data[0]
            bluePressed = data[1]
            extKillState = data[2]
            intKillState = data[3]
            depth_bytes = data[4:8]

            depth = struct.unpack('<f', depth_bytes)[0]

            # Optionally verify trailing \r\n
            if data[8] != 13 or data[9] != 10:
                print("Warning: packet missing expected newline characters")

            return greenPressed, bluePressed, extKillState, intKillState, depth

try:
    while True:
        gp, bp, ek, ik, depth = read_packet()
        print(f"Green: {gp}, Blue: {bp}, ExtKill: {ek}, IntKill: {ik}, Depth: {depth}")

except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
