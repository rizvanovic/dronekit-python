# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math

# Connect to the Vehicle
print("Connecting")
connection_string = '/dev/ttyS0'
wait_ready = input("Wait ready, True or False? \n")
vehicle = connect(connection_string, wait_ready=False,baud=57600)

print(" Autopilot Firmware version: %s" % vehicle.version)

# Display basic vehicle state
#print(" Type: %s" % vehicle._vehicle_type)
#print(" Armed: %s" % vehicle.armed)
print(" System status: %s" % vehicle.system_status.state)
#print(" GPS: %s" % vehicle.gps_0)
#print(" Alt: %s" % vehicle.location.global_relative_frame.alt)
print("Battery: %s" % vehicle.battery)











vehicle.close()
