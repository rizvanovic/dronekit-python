# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal, VehicleMode
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

def change_throttle_old(throttle, timeout):
    msg = vehicle.message_factory.command_long_encode(
        0,
        0,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        209,
        6,
        0,
        throttle, #percentage
        0, #seconds
        6,
        0,
        0
        )
    
    vehicle.send_mavlink(msg)

def change_throttle(throttle):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        0, 0, 0, # x, y, z velocity in m/s
        0, 0, throttle, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    
    vehicle.send_mavlink(msg)

running = True
vehicle.mode = VehicleMode("OFFBOARD")
vehicle.armed = True
while running == True:
    try:
        tpc = int(input("Choose throttle % : "))
        tt = int(input("For how long?: "))
        change_throttle(tpc)
        print(f"Running for {tt} seconds.")
        time.sleep(tt)
    except KeyboardInterrupt:
        vehicle.armed = False
        running = False


vehicle.close()