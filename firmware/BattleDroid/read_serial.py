import serial
import time
import sys

try:
    ser = serial.Serial('COM10', 115200, timeout=2)
    print("Connected to COM10")
    
    # Reset ESP32
    ser.setDTR(False)
    ser.setRTS(True)
    time.sleep(0.1)
    ser.setDTR(True)
    ser.setRTS(False)
    time.sleep(0.1)
    ser.setRTS(True)
    
    # Read for 8 seconds
    start = time.time()
    while time.time() - start < 8:
        line = ser.readline()
        if line:
            print(line.decode('utf-8', errors='replace').strip())
    ser.close()
except Exception as e:
    print(f"Error: {e}")
