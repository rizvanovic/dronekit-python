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
print("Read channels individually:")
print(" Ch1: %s" % vehicle.channels['1'])
print(" Ch2: %s" % vehicle.channels['2'])
print(" Ch3: %s" % vehicle.channels['3'])
print(" Ch4: %s" % vehicle.channels['4'])
print(" Ch5: %s" % vehicle.channels['5'])
print(" Ch6: %s" % vehicle.channels['6'])
print(" Ch7: %s" % vehicle.channels['7'])
print(" Ch8: %s" % vehicle.channels['8'])
print("Number of channels: %s" % len(vehicle.channels))

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









vehicle.close()
