import serial
import time

# Change '/dev/ttyACM0' to the correct port (e.g. 'COM3' on Windows)
serial_port = '/dev/ttyACM0'
baud_rate = 115200

try:
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    time.sleep(2)  # wait for the serial connection to initialize

    print(f"Connected to {serial_port} at {baud_rate} baud.\nReading...\n")
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            print(line)

except serial.SerialException as e:
    print(f"Serial error: {e}")

except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial connection closed.")

