# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math
#from pymavlink.dialects.v10 import ardupilotmega as mavlink1
#from pymavlink.dialects.v20 import ardupilotmega as mavlink2

# Connect to the Vehicle
print("Connecting")
connection_string = '/dev/ttyS0'
#wait_ready = input("Wait ready, True or False? \n")
vehicle = connect(connection_string, wait_ready=False,baud=57600)
#vehicle = mavutil.mavlink_connection(connection_string, baud=57600, dialect = "ardupilotmega")
#print(" Autopilot Firmware version: %s" % vehicle.version)

# Display basic vehicle state
#print(" Type: %s" % vehicle._vehicle_type)
#print(" Armed: %s" % vehicle.armed)
#print(" System status: %s" % vehicle.system_status.state)
#print(" GPS: %s" % vehicle.gps_0)
#print(" Alt: %s" % vehicle.location.global_relative_frame.alt)
#print(" Battery: %s" % vehicle.battery)

input("Press enter to arm. ")

#def PX4setMode(mavMode):
#    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
#                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#                                               mavMode,
#                                               0, 0, 0, 0, 0, 0)

def change_throttle(throttle):
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 0, 1, 0, 0, 0, 0, 0, 0)

    
    vehicle.send_mavlink(msg)


running = True
#PX4setMode(MAV_MODE_OFFBOARD)
time.sleep(1)
armchoice = input("Arm y/n: ")
if armchoice == 'y'
    vehicle.armed = True
while running == True:
    try:
        tpc = int(input("Choose throttle % : "))
       # tt = int(input("For how long?: "))
        change_throttle(tpc)
       # print(f"Running for {tt} seconds.")
        #time.sleep(tt)
    except KeyboardInterrupt:
        vehicle.armed = False
        running = False


vehicle.close()