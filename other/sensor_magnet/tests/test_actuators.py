
import serial
import sys
import time

c = serial.Serial("/dev/ttyACM0", 9600)

if not c.isOpen():
	c.open()

i = 0
while i < 1000:
	c.write("A90,90,90,90")
	time.sleep(0.01)


while i < 1000:
    for read in c.read():
        sys.stdout.write(read)
    i += 1