import Jetson.GPIO as GPIO
import smbus
import time

# Initialize I2C bus 7 and device address
I2C_BUS = 7
DEVICE_ADDRESS = 0x48  # Replace with your I2C device address
bus = smbus.SMBus(I2C_BUS)

# Setup GPIO pin for any additional control you need (like reset, power, etc.)
#GPIO_PIN = 18  # Example GPIO pin, change to what you need
#GPIO.setmode(GPIO.BOARD)
#GPIO.setup(GPIO_PIN, GPIO.OUT)

# Function to toggle GPIO pin (example for resetting the device)
#def reset_device():
    #GPIO.output(GPIO_PIN, GPIO.LOW)  # Set pin low to reset
    #time.sleep(0.5)
    #GPIO.output(GPIO_PIN, GPIO.HIGH)  # Set pin high to resume
    #print("Devi
    # ce has been reset")

def write_to_device(register, value):
    """Writes a byte to the I2C device."""
    try:
        bus.write_byte_data(DEVICE_ADDRESS, register, value)
        print(f"Data {value} written to register {register}.")
    except Exception as e:
        print(f"Error: {e}")

def read_from_device(register):
    """Reads a byte from the I2C device."""
    try:
        value = bus.read_byte_data(DEVICE_ADDRESS, register)
        print(f"Data read from register {register}: {value}")
        return value
    except Exception as e:
        print(f"Error: {e}")
        return None

if __name__ == "__main__":
    try:
        # Optionally reset the device using GPIO
        #reset_device()

        # Write to the I2C device
        register_to_write = 0x01  # Replace with the register you want to write to
        value_to_write = 0x10     # Example value to write
        write_to_device(register_to_write, value_to_write)

        # Small delay for the device to process
        time.sleep(0.1)

        # Read from the I2C device
        register_to_read = 0x01  # Replace with the register you want to read from
        read_from_device(register_to_read)

    finally:
        # Cleanup GPIO settings to prevent issues
        GPIO.cleanup()