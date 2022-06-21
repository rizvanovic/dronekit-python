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
print(" Battery: %s" % vehicle.battery)

input("Press enter to arm. ")

vehicle.armed = True
#cmds = vehicle.commands
#cmds.clear()
running = True
while running == True:
    try:
        pwm = int(input("Choose PWM (ALL CHANNELS): "))
        vehicle.channels.overrides['1'] = pwm
        vehicle.channels.overrides['2'] = pwm
        vehicle.channels.overrides['3'] = pwm
        vehicle.channels.overrides['4'] = pwm
        vehicle.channels.overrides['5'] = pwm
        vehicle.channels.overrides['6'] = pwm
        time.sleep(1)
    except KeyboardInterrupt:
        vehicle.channels.overrides['1'] = None
        vehicle.channels.overrides['2'] = None
        vehicle.channels.overrides['3'] = None
        vehicle.channels.overrides['4'] = None
        vehicle.channels.overrides['5'] = None
        vehicle.channels.overrides['6'] = None
        vehicle.armed = False
        running = False

input("Press enter to disarm. ")








vehicle.close()
