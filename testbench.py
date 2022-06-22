# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal, VehicleMode
from pymavlink import mavutil
import time, sys, argparse, math
from pymavlink.dialects.v10 import ardupilotmega as mavlink1
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

# Connect to the Vehicle
print("Connecting")
connection_string = '/dev/ttyS0'
wait_ready = input("Wait ready, True or False? \n")
#vehicle = connect(connection_string, wait_ready=False,baud=57600)
vehicle = mavutil.mavlink_connection(connection_string, baud=57600, dialect = "ardupilotmega")
print(" Autopilot Firmware version: %s" % vehicle.version)

# Display basic vehicle state
#print(" Type: %s" % vehicle._vehicle_type)
#print(" Armed: %s" % vehicle.armed)
print(" System status: %s" % vehicle.system_status.state)
#print(" GPS: %s" % vehicle.gps_0)
#print(" Alt: %s" % vehicle.location.global_relative_frame.alt)
print(" Battery: %s" % vehicle.battery)

input("Press enter to arm. ")

def change_throttle(throttle, timeout):
    msg = mavlink2.MAV_CMD_DO_MOTOR_TEST(
        6,
        0,
        throttle, #percentage
        timeout, #seconds
        6,
        0
    )
    vehicle.send_mavlink(msg)
    
#vehicle.send_mavlink(msg)

running = True
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while running == True:
    try:
        tpc = int(input("Choose throttle % : "))
        tt = int(input("For how long?: "))
        change_throttle(tpc,tt)
        time.sleep(1)
    except KeyboardInterrupt:
        vehicle.armed = False
        running = False


msg = mavlink2.MAV_CMD_DO_MOTOR_TEST()



vehicle.close()