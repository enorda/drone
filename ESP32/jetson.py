# Jetson Code #
import time
import json
from serial import Serial

# Adjust this to the correct port for your setup
PORT = 'COM4'  # Change to your actual port
BAUDRATE = 115200  # Ensure this matches the ESP32 baud rate

def main():
    # Example data
    packet_number = 1
    gps_coordinates = [12.345, 67.890]  # Replace with actual GPS coordinates
    move_up = 1  # Change to actual condition as needed

    # Set up the serial connection
    with Serial(PORT, BAUDRATE, timeout=1) as ser:
        time.sleep(2)  # Allow time for the connection to establish
        while True:
            # Create the data dictionary
            data_dict = {
                "packet_number": packet_number,
                "gps_coordinates": gps_coordinates,
                "move_up": move_up
            }
            
            # Convert the dictionary to a JSON string
            data_to_send = json.dumps(data_dict)
            
            # Send the JSON string over serial
            ser.write(data_to_send.encode('utf-8'))
            print(f"Sent: {data_to_send}")
            
            time.sleep(2)  # Wait before sending the next message

if __name__ == "__main__":
    main()
