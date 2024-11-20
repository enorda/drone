# raspberry pi #
import time
import json
from serial import Serial

PORT = 'COM3'
BAUDRATE = 115200

def main():
    with Serial(PORT, BAUDRATE, timeout=1) as ser:
        time.sleep(2)
        print("Waiting for data from ESP32...")
        while True:
            data = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"Raw data received: {data}")
            try:
                if data.startswith("Received: "):
                    clean_data = data[len("Received: "):].strip()
                    print(f"Cleaned data: {clean_data}")
                    received_data = json.loads(clean_data)
                    print(f"Received valid data: {received_data}")
                else:
                    print("Unexpected data format.")
            except json.JSONDecodeError as e:
                print(f"Error parsing JSON data: {e}")
            time.sleep(1)

if __name__ == "__main__":
    main()
