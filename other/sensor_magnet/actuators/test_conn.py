
import serial
import sys

c = serial.Serial("/dev/ttyACM0", 9600)

if not c.isOpen():
	c.open()

c.write("A0,0,0,0")

i = 0
while i < 1000:
    for read in c.read():
        sys.stdout.write(read)
    i += 1