#!/usr/bin/env python
import struct
import io
import time

path = "/dev/input/js1"

EVENT_SIZE = struct.calcsize("IhBB")
f = io.open(path,mode="rb")
event = f.read(EVENT_SIZE)

while event or 1:
    #print(struct.unpack("IhBB", event))

	#struct timeval time;
	#unsigned short type;
	#unsigned short code;
	#unsigned int value;

    (tv_msec,  value, presstype, number) = struct.unpack("IhBB", event)

    if presstype==1:
        if value == 0:
            print("you released A")
        if value == 1:
            print("you pressed A")

    time.sleep(0.1)
    event = f.read(EVENT_SIZE)
    
