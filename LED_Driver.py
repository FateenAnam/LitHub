from time import sleep
import serial

# Connect to Arduino
# On MacOS, find the port by "ls /dev/*" in terminal. Then, copy in /dev/tty.usbmodem or /dev/tty.usbserial.
dev = serial.Serial(port="/dev/tty.usbmodem143301", baudrate=9600)

print("Starting")
sleep(3)
print("Ready")

while True:
    command = input('input int: ')
    dev.write(command.encode('ascii'))
    print("Running")