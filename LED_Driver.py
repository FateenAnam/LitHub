from time import sleep

import serial

dev = serial.Serial("COM1", baudrate=9600)

print('starting')
sleep(3)
print('ready')
running = True
while (running == True):
    command = input('input int: ')
    dev.write(command.encode('ascii'))
    print('running')