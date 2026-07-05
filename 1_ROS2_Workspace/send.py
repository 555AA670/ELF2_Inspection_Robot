# -*- coding: utf-8 -*-
import serial
import time
import os
import argparse
import crcmod.predefined

SOH = b'\x01'
STX = b'\x02'
EOT = b'\x04'
ACK = b'\x06'
NAK = b'\x15'
CAN = b'\x18'
C = b'C'

def crc16_ccitt(data):
    crc16 = crcmod.predefined.Crc('xmodem')
    crc16.update(data)
    return crc16.digest()

def send_packet(ser, packet_num, data):
    packet_num_byte = bytes([packet_num % 256])
    packet_num_inv = bytes([255 - (packet_num % 256)])
    header = SOH if len(data) == 128 else STX
    packet = header + packet_num_byte + packet_num_inv + data + crc16_ccitt(data)
    ser.write(packet)
    
    while True:
        resp = ser.read(1)
        if resp == ACK:
            return True
        elif resp == NAK:
            return False
        elif resp == CAN:
            raise Exception("Canceled by receiver")
        else:
            if resp:
                print(f"   [DEBUG] Ignoring byte while waiting for ACK: {resp.hex()}")

def send_file(ser, file_path):
    print("\nSending 'update' trigger to wake up App...")
    # WE MUST SEND THE NEWLINE HERE!
    ser.write(b"update\n")
    time.sleep(0.5)

    print("\nWaiting for receiver's start signal ('C')...")
    
    # Read and ignore everything until the Bootloader actually starts
    # This prevents mistaking the 'C' in 'RTC' telemetry for the Y-Modem start signal
    while True:
        line = ser.readline()
        if b"Waiting for Y-Modem update..." in line:
            break
        if b"Bootloader V2" in line:
            pass # Keep reading
            
    while True:
        if ser.read(1) == C:
            break

    file_size = os.path.getsize(file_path)
    file_name = os.path.basename(file_path).encode('utf-8')
    
    block0_data = file_name + b'\x00' + str(file_size).encode('utf-8')
    block0_data = block0_data.ljust(128, b'\x00')
    
    print(f"Sending Header (Block 0): {file_name.decode('utf-8')}, {file_size} bytes")
    while not send_packet(ser, 0, block0_data):
        pass

    # Wait for ACK and C
    if ser.read(1) != C:
        while True:
            r = ser.read(1)
            if r == C: break
            if r == ACK: continue
    
    packet_num = 1
    sent_bytes = 0
    with open(file_path, 'rb') as f:
        while True:
            chunk = f.read(1024)
            if not chunk:
                break
            
            chunk = chunk.ljust(1024, b'\x1A')
            while not send_packet(ser, packet_num, chunk):
                print("Retrying block...")
            
            sent_bytes += 1024
            packet_num += 1
            print(f"\rProgress: {min(sent_bytes, file_size)} / {file_size} bytes sent", end="", flush=True)

    print("\nSending EOT...")
    ser.write(EOT)
    while ser.read(1) != ACK:
        ser.write(EOT)
        time.sleep(0.1)

    print("Sending End Header (Empty Block 0)...")
    empty_block = b'\x00' * 128
    send_packet(ser, 0, empty_block)
    print("Transfer Complete!")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('file', help="Path to firmware (.bin) file")
    parser.add_argument('--port', required=True, help="Serial port (e.g., COM5)")
    parser.add_argument('--baudrate', type=int, default=115200, help="Baud rate")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baudrate, timeout=1)
    try:
        send_file(ser, args.file)
    except KeyboardInterrupt:
        print("Canceled.")
    finally:
        ser.close()
