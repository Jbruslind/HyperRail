"""Basic Command Interpeter for driving the ESP 32 over serial"""

from os import write
import serial
from time import sleep

# Baud Rate to communicate to the ESP 32 over
BAUD_RATE = 115200

# Set to whatever device the ESP 32 controller is opperating on
DEVICE_PATH = "/dev/serial0"


def initiateSerialConnection(s: serial.Serial):
    """Initate communication with ESP32"""

    # Send wake-up signal to the ESP 32
    print("Sending Wake-up packet to the controller...")
    s.write("\r\n\r\n")

    # Wait a few seconds for it to initialize
    sleep(3)
    print("Controller Initialized!")

    # Flush the serial input to prepare for the next set of data
    s.flushInput()

def goToXY(s: serial.Serial):
    """Go to a specified X Y Coordinate"""

    x = input("Input the X Coordinate (mm): ")
    y = input("Input the Y Coordinate (mm): ")
    speed = input("Input the feed rate (mm/min): ")
    s.write(f"G1 X{x} Y{y} F{speed}\n")
    print("GRBL Response: " + s.readline().strip())

def sendGCode(s: serial.Serial):
    """Send a single line of GCode to the driver"""
    line = input("G-code to send: ").strip()
    s.write(line + "\n")
    print("GRBL Response: " + s.readline().strip())

def homeMachine(s: serial.Serial):
    """Home machine's axis"""
    s.write("$H\n")
    print("GRBL Response: " + s.readline().strip())

def feedGCodeFile(s: serial.Serial):
    """Feed a given G-code file into the controller"""
    file_path = input("Path to g-code: ")

    # Open the G code file
    with open(file_path) as f:

        # Loop through every line in the g-code file and send it to the controller
        for line in f:
            l = line.strip()
            print("Sending: " + l)
            s.write(l + "\n")
            print("GRBL Response: " + s.readline().strip())

def listCommands(s: serial.Serial):
    """List all available commands that can be used on the driver"""
    s.write("$cmd\n")
    print("Commands: ")
    for line in s.readlines():
        print(line)

def genericCommand(s: serial.Serial):
    """Send another command that is not listed in the options"""
    user_input = input("Input the command to send to the driver (settings must start with a $): ").strip()
    s.write(user_input)
    print("GRBL Response: ")
    for line in s.readlines():
        print(line)

    
def menu(s: serial.Serial):
    """Display a menu with options to control the driver board"""
    print("Welcome to a very basic driver interface for commanding an ESP 32 controller!\n")
    print("1) Go To X-Y")
    print("2) Feed G-Code file")
    print("3) Send G-Code")
    print("4) Home Machine")
    print("5) List All Commands")
    print("6) Send Other Commands\n")
    user_input = input("Current Selection: ").strip()

    # Convert the input into actual function mappings
    if user_input == "1":
        goToXY(s)
    elif user_input == "2":
        feedGCodeFile(s)
    elif user_input == "3":
        sendGCode(s)
    elif user_input == "4":
        homeMachine(s)
    elif user_input == "5":
        listCommands(s)
    elif user_input == "6":
        genericCommand(s)


if __name__ == "__main__":
    # Create the serial device to communicate over
    deviceSerial = serial.Serial(DEVICE_PATH, BAUD_RATE)

    initiateSerialConnection(deviceSerial)
    try:
        while True:
            menu(None)
    except KeyboardInterrupt    :
        print("\nCtrl-C terminated the program")
        deviceSerial.close()
        pass
