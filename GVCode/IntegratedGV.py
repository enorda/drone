from __future__ import print_function
import time
import math
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative
import serial  
import logging

#################
# Logger Setup 
#################
logging.basicConfig(
    filename='gv.log',
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("GVLogger")

#################
# Config / CLI
#################
parser = argparse.ArgumentParser(description="Single-process Ground Vehicle control.")
parser.add_argument("--connection", default="udp:127.0.0.1:14550",
                    help="DroneKit connection string (default is SITL).")
parser.add_argument("--espport", default="/dev/serial0",  
                    help="ESP32 serial port (default: /dev/serial0).")
parser.add_argument("--espbaud", type=int, default=115200,
                    help="ESP32 baudrate (default: 115200).")
args = parser.parse_args()

#######################
# Vehicle Connection
#######################
def connect_ground_vehicle(conn_str):
    """
    Connects to the ground vehicle (ArduPilot Rover firmware) via DroneKit.
    """
    print(f"Connecting to ground vehicle on {conn_str}...")
    vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True) 
    return vehicle

def arm_ground_vehicle(vehicle):
    """
    Arms the ground vehicle and sets mode to GUIDED.
    """
    print("Waiting for vehicle to become armable...")
    while not vehicle.is_armable:
        time.sleep(1)

    print("Arming motors (Ground Vehicle).")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Vehicle is armed and ready in GUIDED mode.")

#############################
# Dynamic Waypoint Driving
#############################
def get_distance_meters(loc1, loc2):
    """
    Approximate ground distance in meters between two LocationGlobalRelative objects.
    """
    dlat = loc2.lat - loc1.lat
    dlon = loc2.lon - loc1.lon
    return math.sqrt((dlat * 111139)**2 + (dlon * 111139)**2)

def drive_to_waypoint(vehicle, ser, close_threshold=1.0, speed=3.0):
    """
    Continuously moves the GV to waypoints from ESP32.
    If a new waypoint arrives before reaching the current one, it updates the destination immediately.
    """
    current_waypoint = None

    while True:
        # Check for new waypoint while moving
        new_waypoint = read_esp_line(ser)
        if new_waypoint:
            current_waypoint = new_waypoint  # Overwrite with latest waypoint

        if current_waypoint:
            lat, lon = current_waypoint
            print(f"Driving to latest waypoint: lat={lat}, lon={lon}")

            target_location = LocationGlobalRelative(lat, lon, 0)
            vehicle.groundspeed = speed
            vehicle.simple_goto(target_location)

            while True:
                current_loc = vehicle.location.global_relative_frame
                dist = get_distance_meters(current_loc, target_location)
                print(f" Distance to waypoint: {dist:.2f} m")

                # If we get a new waypoint while moving, update it immediately
                new_waypoint = read_esp_line(ser)
                if new_waypoint:
                    print("New waypoint received, updating route...")
                    current_waypoint = new_waypoint  # Update to latest
                    break  # Exit the loop to start moving to new waypoint

                if dist <= close_threshold:
                    print("Reached waypoint.")
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
        if line:
            print("Received from ESP32:", line)
            logger.info(f"ESP32 inbound: {line}")

            # If it starts with WAYPOINT=..., parse
            if line.startswith("WAYPOINT="):
                coords_str = line.split("=", 1)[1]  # e.g. "37.4221,-122.0841"
                parts = coords_str.split(',')
                if len(parts) == 2:
                    lat = float(parts[0])
                    lon = float(parts[1])
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
    # 1) Connect to the ground vehicle
    vehicle = connect_ground_vehicle(args.connection)
    arm_ground_vehicle(vehicle)

    # 2) Connect to the ESP32
    ser = connect_esp32(args.espport, args.espbaud)
    if not ser:
        print("Failed to connect to ESP32. Exiting gv_control.")
        return

    print("Ready to receive waypoints from ESP32...")

    try:
        # Use the modified function to follow waypoints dynamically
        drive_to_waypoint(vehicle, ser, close_threshold=1.0, speed=3.0)

    except KeyboardInterrupt:
        print("User interrupted.")
    except Exception as e:
        print(f"Error in gv_control loop: {e}")
    finally:
        print("Stopping vehicle and closing connections...")
        vehicle.mode = VehicleMode("HOLD")
        time.sleep(2)
        vehicle.close()

        if ser and ser.is_open:
            ser.close()

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
