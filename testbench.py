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
time.sleep(2.5)
#vehicle = mavutil.mavlink_connection(connection_string, baud=57600, dialect = "ardupilotmega")
#print(" Autopilot Firmware version: %s" % vehicle.version)
# Display basic vehicle state
#print(" Type: %s" % vehicle._vehicle_type)
#print(" Armed: %s" % vehicle.armed)
#print(" System status: %s" % vehicle.system_status.state)
#print(" GPS: %s" % vehicle.gps_0)
#print(" Alt: %s" % vehicle.location.global_relative_frame.alt)
#print(" Battery: %s" % vehicle.battery)

print(f"Aerit test bench, connected to vehicle operating PX4 firmware {vehicle.version}. \n Battery: {vehicle.battery}. ")
def change_throttle(throttle, instance, motor_count):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, # command
        0, # confirmation
        instance, # instance
        0, # throttle type
        throttle, # throttle value
        0, # timeout
        motor_count, # motor count 
        0, # test order
        0  # empty
        )

    
    vehicle.send_mavlink(msg)


running = True
sttngchce = 'n'
stepsize = 5
#PX4setMode(MAV_MODE_OFFBOARD)
time.sleep(1)
input("Press enter to start. ")
while running == True:

    try:
        if sttngchce == 'n':
            #iNst = int(input("Instance: "))
            motcnt = int(input("Motor count: "))
            sttngche = 'y'
        tpc = int(input("Choose throttle % : "))
        psteps = tpc % stepsize
        print(psteps)
        restpow = tpc - (psteps * stepsize)
        print(restpow)
        while True:
            for pwi in range(psteps):
                for iNst in range(6):
                    if pwi == 5:
                        change_throttle((pwi+1)*stepsize + restpow, iNst + 1, motcnt)
                    else:

                        change_throttle((tpc+1), iNst + 1, motcnt)
                    time.sleep(2)
            time.sleep(0.25)


            
    except KeyboardInterrupt:
        print("\n\nKeyboard interrupt: STOPPING NOW.\n\n")
        for iNst in range(6):
                change_throttle(0, iNst + 1, motcnt)
        time.sleep(1)
        runchoice = input("Would you like to run again? [y/n] ")
        if runchoice == 'n':
            running = False
        elif runchoice == 'y':
            sttngchce = input("Same instance and motor count? [y/n]")



vehicle.close()