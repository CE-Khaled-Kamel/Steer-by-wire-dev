import socket 
import time
from ctypes import windll
import math
import struct

a = 40.0
b = 150.0
speed = 0
steeringAngle = 0.0
friction = 1.0

loopTimeStep = 20e-3
cycles = 0

def getSpeed(c):
    return a + (b-a)*min(c*loopTimeStep / 10.0 ,1)

def getAngle(c):
    return 15.0*math.sin(2*math.pi*0.5*c*loopTimeStep)

Host = '127.0.0.1'
Port = 8080
# Create a new socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Connect to the remote socket
s.connect((Host, Port))

# The Dyno Scenario
while cycles < 1000 :
    start = time.perf_counter()
    
    if cycles == 500:
        friction = 0.3
    elif cycles == 650 :
        friction = 1.0

    speed = getSpeed(cycles)
    steeringAngle = getAngle(cycles)
    telemetry = struct.pack('<fff', speed, steeringAngle, friction)
    
    # Send data to the socket
    s.sendall(telemetry)

    cycles+=1
    end = time.perf_counter()
    elapsed = start - end
    if elapsed > loopTimeStep :
        print("Dyno cycle is lagging, Python Exiting...")
        exit(1)
    else:
        windll.winmm.timeBeginPeriod(1)
        time.sleep(loopTimeStep - elapsed)
        windll.winmm.timeEndPeriod(1)
print("DONE!")