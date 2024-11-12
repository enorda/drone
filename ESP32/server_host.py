import time
from serial import Serial

# Adjust this to the correct port for your setup
PORT = '/dev/ttyCH341USB0'  # Change to your actual port
BAUDRATE = 115200      # Make sure this matches the ESP32 baud rate

def main():
    x = "[packet1, gps_coordinates, moveup]"
    # Set up the serial connection
    with Serial(PORT, BAUDRATE, timeout=1) as ser:
        time.sleep(2)  # Give time for the connection to establish
        while True:
            # Prepare the data you want to send
            data_to_send = x  # Example data
            ser.write(data_to_send.encode())  # Send data
            print(f"Sent: {data_to_send.strip()}")
            time.sleep(2)  # Wait before sending the next message

if __name__ == "__main__":
    main()
