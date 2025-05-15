from __future__ import print_function
import time
import math
import argparse
from dronekit import *
#from dronekit import connect, VehicleMode, LocationGlobalRelative
import re
import serial
from gpiozero import Servo
import logging
from time import sleep
import os
from datetime import datetime
import sys

#################
# Logger Setup 
#################
GV_log_folder = "GV_Logging Folder"
# Ensure the log folder exists
if not os.path.exists(GV_log_folder):
    os.makedirs(GV_log_folder)
GV_text_folder = "GV_Text Folder"
if not os.path.exists(GV_text_folder):
    os.makedirs(GV_text_folder)


# Custom log level setup for AVC
AVC_LOG = 25  # Pick a value that does not clash with existing levels
logging.addLevelName(AVC_LOG, "AVC")

def log_avc(self, message, *args, **kwargs):
    if self.isEnabledFor(AVC_LOG):
        self._log(AVC_LOG, message, args, **kwargs)

logging.Logger.avc = log_avc

# Custom level filter
class CustomLevelFilter(logging.Filter):
    def filter(self, record):
        return record.levelno == AVC_LOG

# Function to generate a unique filename
def get_unique_filename(base_filename):
    """
    This function checks if the file already exists.
    If it does, it appends a number to the filename to make it unique.
    """
    file_name, file_extension = os.path.splitext(base_filename)
    counter = 0
    new_filename = base_filename
    while os.path.exists(new_filename):
        new_filename = f"{file_name}_{counter}{file_extension}"
        counter += 1
    return new_filename

current_datetime = datetime.now().strftime("%m-%d-%y %I:%M")
log_filename = get_unique_filename(os.path.join(GV_log_folder,f"{current_datetime} GV.log"))

# Check and get unique filename for the output text file
unique_output_filename = get_unique_filename(os.path.join(GV_text_folder, f"{current_datetime} GV_output.log"))

# Set up logging to flight.log
logging.basicConfig(
    filename=log_filename,  # Log file for logger.avc
    level=AVC_LOG,  # Set to the custom level
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# Add a custom filter to the root logger
root_logger = logging.getLogger()
for handler in root_logger.handlers:
    handler.addFilter(CustomLevelFilter())

# Create a logger for FlightLogger
logger = logging.getLogger("GVLogger")

# Custom stream class to write to both terminal and file
class Output:
    def __init__(self, filename, mode="w"):
        self.terminal = sys.stdout
        self.file = open(filename, mode)

    def write(self, message):
        
        if message.strip():  # Ensure we don’t log empty lines with timestamps
            timestamp = f"\033[96m[{datetime.now().strftime('%m-%d-%y - %I:%M:%S')}]\033[0m "
            self.terminal.write(timestamp + message + "\n")
            self.file.write(f"[{datetime.now().strftime('%m-%d-%y - %I:%M:%S')}] {message}\n")

    def flush(self):
        self.terminal.flush()
        self.file.flush()


# Redirect sys.stdout to the custom Output class
sys.stdout = Output(unique_output_filename)
sys.stderr = sys.stdout

#################
# SERVO SETUP
#################
correction = 0.0
maxPW = (2.0+correction)/1000
minPW = (1.0+correction)/1000
servo = Servo(14, min_pulse_width = minPW, max_pulse_width = maxPW)

#################
# Config / CLI
#################
parser = argparse.ArgumentParser(description="Single-process Ground Vehicle control.")
parser.add_argument("--connection", default="udp:127.0.0.1:14550",
                    help="DroneKit connection string (default is SITL).")
parser.add_argument("--espport", default="/dev/ttyUSB0",  
                    help="ESP32 serial port (default: /dev/serial0).")
parser.add_argument("--espbaud", type=int, default=115200,
                    help="ESP32 baudrate (default: 115200).")
args = parser.parse_args()


def connect_ground_vehicle(conn_str):
    """
    Connects to the ground vehicle (ArduPilot Rover firmware) via DroneKit.
    """
    print(f"Connecting to ground vehicle on {conn_str}...")
    vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True) 
    return vehicle

def arm_ground_vehicle(vehicle):
    """
    Arms the ground vehicle and sets mode to HOLD.
    """
    print("Waiting for vehicle to become armable...")
    while not vehicle.is_armable:
        time.sleep(1)

    print("Arming motors (Ground Vehicle).")
    vehicle.mode = VehicleMode("HOLD")
    print("Vehicle has armed in HOLD")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    logger.avc("Arming motors (Ground Vehicle.)")
    print("Vehicle is armed and ready in HOLD mode.")

def get_distance_meters(loc1, loc2):
    """
    Approximate ground distance in meters between two LocationGlobalRelative objects.
    """
    dlat = loc2.lat - loc1.lat
    dlon = loc2.lon - loc1.lon
    return math.sqrt((dlat * 111139)**2 + (dlon * 111139)**2)



def drive_to_waypoint(vehicle, ser, close_threshold=1):
    """
    Continuously moves the GV to waypoints from ESP32.
    If a new waypoint arrives before reaching the current one, it updates the destination immediately.
    """
    current_waypoint = None
    #new_waypoint = {32.9871337, -96.7495181} #Hard code testing
    while True:
        # Check for new waypoint while moving
        new_waypoint = read_esp_line(ser)
        if new_waypoint:
            current_waypoint = new_waypoint  # Overwrite with latest waypoint
            new_waypoint = None
            lat, lon = current_waypoint
            print(f"Driving to latest waypoint: lat={lat}, lon={lon}")
            logger.avc("Scout Found Marker. Driving to latest waypoint.")
            target_location = LocationGlobalRelative(lat, lon, 0)
       # if current_waypoint:
          #  lat, lon = current_waypoint
          #  print(f"Driving to latest waypoint: lat={lat}, lon={lon}")

                    # Get the set of commands from the vehicle
            cmds = vehicle.commands
            cmds.download()
            cmds.wait_ready()
            
            cmds.clear()
            # Use the modified function to follow waypoints dynamically
            # Create and add commands
            
            
            #cmd1=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, 0, 0, 32.9945609, -96.7518674, 0)
           # cmd1=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat+0.00001818, lon, 0)
            cmd2=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, 0)
           # cmd3=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0)
            #cmds.add(cmd1)
            cmds.add(cmd2)
            #cmds.add(cmd3)
            cmds.upload() # Send commands
            cmds.wait_ready()
            time.sleep(5)
            logger.avc("Driving To Waypoint")
            vehicle.mode = VehicleMode("AUTO")


            while True:
                current_loc = vehicle.location.global_relative_frame
                dist = get_distance_meters(current_loc, target_location)
                speed = vehicle.groundspeed
                print(f" Distance to waypoint: {dist:.2f} m")
                print(f"Movement Speed: {speed:.2f} m")
             #   print({vehicle.groundspeed()})
                #FOR TESTING SERVO:
                # dist = 0
                ########

                # If we get a new waypoint while moving, update it immediately
              #  new_waypoint = read_esp_line(ser)
                
                if new_waypoint:
                    print("New waypoint received, updating route...")
                    current_waypoint = new_waypoint  # Update to latest
                    break  # Exit the loop to start moving to new waypoint

                if dist <= close_threshold and speed <= 0.01:
                    print("Reached waypoint.")
                    logger.avc("Reached waypoint")
                    # reached waypoint, release package
                    servo.min()
                    logger.avc("Payload detaching")
                    print("Releasing MedKit")
                    sleep(1)
                    servo.value = None
                    
                    cmds.clear()
                    cmd3=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat+0.00001818, lon+0.00001818, 0)
                    cmds.add(cmd3)
                    cmds.upload() # Send commands
                    cmds.wait_ready()
                    sleep(1)
                    vehicle.mode = VehicleMode("AUTO")
                    sleep(10)
                    current_waypoint = None  # Clear waypoint since we reached it
                    break

                time.sleep(1)




###################################
# ESP32 Serial Communication
###################################
def connect_esp32(port, baudrate):
    """
    Connect to the ESP32 on the Raspberry Pi's UART serial port.
    """
    try:
        print(f"Connecting to ESP32 on {port} at {baudrate} baud...")
        ser = serial.Serial(port, baudrate, timeout=1)  # PySerial for Raspberry Pi
        time.sleep(2)  # Give ESP32 time to initialize
        if ser.is_open:
            print("ESP32 connection established.")
            return ser
        else:
            print("Failed to open ESP32 serial port.")
            ser.close()
            return None
    except Exception as e:
        print(f"Error connecting to ESP32: {e}")
        return None

def read_esp_line(ser):
    """
    Reads one line from the ESP32 (with a small timeout).
    Expects format: "WAYPOINT=37.4221,-122.0841"
    Returns (lat, lon) if parse is successful, else None.
    """
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        print("waiting on server-side esp32")
        if line:
            if line.startswith("Received:"):
                line = line[len("Received: "):].strip()
                line = "WAYPOINT=" + line
            print("Received from ESP32:", line)
            logger.info(f"ESP32 inbound: {line}")
            print(line)

            # If it starts with WAYPOINT=..., parse
            if line.startswith("WAYPOINT="):
                coords_str = line[line.find("[") + 1 : line.find("]")]
                # coords_str = re.findall(r'-\d+\.\d+', line)
                lat, lon = map(float, coords_str.split(","))
                lat = lat+0.00005454
                lon = lon+0.00005454
                print(line)
                #coords_str = line.split("=", 1)[1]  # e.g. "37.4221,-122.0841"
                #parts = coords_str.split(',')
                #if len(parts) == 2:
                    #lat = float(parts[0])
                    #lon = float(parts[1])
                return (lat, lon)
    except Exception as e:
        print(f"Error reading from ESP32: {e}")
    return None


###########################
# GV Control (Main Loop)
###########################
def gv_control():
    """
    Controls the Ground Vehicle:
      1) Connects and arms the rover
      2) Connects to ESP32
      3) Continuously follows the latest waypoint sent from ESP32
    """
    # 1) Connect to the ground vehicle, ensure servo closed
    print("Closing servo")
    #servo.value = 1
    servo.max()
    sleep(1)
    servo.value = None
    vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True) 
    

    
    
    arm_ground_vehicle(vehicle)
    # 2) Connect to the ESP32
    
    ser = connect_esp32(args.espport, args.espbaud)
    if not ser:
        print("Failed to connect to ESP32. Exiting gv_control.")
        return

    print("Ready to receive waypoints from ESP32...")


    try:
        drive_to_waypoint(vehicle, ser)

            
    except KeyboardInterrupt:
        print("User interrupted.")
    except Exception as e:
        print(f"Error in gv_control loop: {e}")
    finally:
        print("Stopping vehicle and closing connections...")
        vehicle.mode = VehicleMode("HOLD")
        time.sleep(2)
        vehicle.close()

        ''' if ser and ser.is_open:
            ser.close()
        '''
        print("Done. Exiting gv_control.")

###################
# Main Entry Point
###################
def main():
    """
    Entry point: just call gv_control().
    """
    gv_control()

if __name__ == "__main__":
    main()
