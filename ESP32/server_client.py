import time
from serial import Serial

# Adjust this to the correct port for your setup
PORT = '/dev/ttyUSB0'  # Change to your actual port
BAUDRATE = 115200      # Make sure this matches the ESP32 baud rate

def main():
    
    # Set up the serial connection
    with Serial(PORT, BAUDRATE, timeout=1) as ser:
        time.sleep(2)  # Give time for the connection to establish
        while True:
            data = ser.readline()
            if data: 
                print(data) 
            time.sleep(1)

if __name__ == "__main__":
    main()
