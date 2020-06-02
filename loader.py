import serial
import os
import os.path
import io
import sys
import fcntl
from time import time, sleep

#-----------------------------------------------
#   SERIAL HAND SHAKING
#-----------------------------------------------

while (os.path.exists("/dev/ttyUSB0") == False ):
	print "WAITING for device USB0"
	sleep(1)

print "Device initializing"
s=serial.Serial('/dev/ttyUSB0',115200,timeout=1) 

#sleep(1) #wait initializing handshake

while (s.inWaiting()>0 and s.outWating>0):
	print "flushing DATA"
	s.flushInput()
	s.flushOutput()

p = s.read(1)
while (p  != '$'):
	p = s.read(1)
	sleep(0.5)
	#s.flushInput()
	print p
	print "WAITING for connection"
	s.write("^")
	sleep(0.1)
	if (p == '*'):
		os.system("/home/QV2/./stopper.sh")
		print "rebooting"	
print "Sending a connection character"
s.write("#") #send a connected char
sleep(1)
s.close() #close connection before loading program
#load the master program
os.system("/home/QV2/./runner.sh")
sleep(2)
