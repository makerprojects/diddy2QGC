#!/usr/bin/env python
'''
Connect DiddyBorg via mavlink with QGroundControl

Copyright Gregor Schlechtriem 2022
Released under GNU LGPL version 3 or later
'''

# Load library functions we want
import time
import os
import sys
import ThunderBorg3
import socket
import struct

# Import mavutil
from pymavlink import mavutil
from threading import Thread

# Settings for the joystick
axisUpDown = 1                          # Joystick axis to read for up / down position
axisUpDownInverted = False              # Set this to True if up and down appear to be swapped
axisLeftRight = 2                       # Joystick axis to read for left / right position
axisLeftRightInverted = False           # Set this to True if left and right appear to be swapped
buttonSlow = 8                          # Joystick button number for driving slowly whilst held (L2)
slowFactor = 0.5                        # Speed to slow to when the drive slowly button is held, e.g. 0.5 would be half speed
buttonFastTurn = 9                      # Joystick button number for turning fast (R2)
interval = 0.00                         # Time between updates in seconds, smaller responds faster but uses more processor time

# Power settings
voltageIn = 12.0                        # Total battery voltage to the ThunderBorg
voltageOut = 12.0 * 0.95                # Maximum motor voltage, we limit it to 95% to allow the RPi to get uninterrupted power

# Setting for switch function
switchOnMin = 1750                      # Minimum pwm signal to indicate switch is on
buttonFastTurn = 3                      # fast turn assigned assigned to channel 3
buttonSlow = 4                          # slow move assigned to channel 4

# functions
def handle_Messages(msg):
    global UdpMavLink
    global driveLeft
    global driveRight
    if msg.get_msgId() == mavutil.mavlink.MAVLINK_MSG_ID_COMMAND_LONG:
        UdpMavLink.mav.mission_ack_send(1,1,mavutil.mavlink.MAV_RESULT_ACCEPTED)
    elif msg.get_msgId() == mavutil.mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        UdpMavLink.mav.param_value_send(b"CRUISE_SPEED    ",5.0,mavutil.mavlink.MAV_PARAM_TYPE_REAL32,2,0)
        UdpMavLink.mav.param_value_send(b"CRUISE_THROTTLE ", 95.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32, 2, 1)
    elif msg.get_msgId() == mavutil.mavlink.MAVLINK_MSG_ID_MANUAL_CONTROL:
        leftRight = float(msg.x) / 1000
        upDown = -float(msg.y) / 1000
        # Determine the drive power levels
        driveLeft = -upDown
        driveRight = -upDown
        if leftRight < -0.05:
            # Turning left
            driveLeft *= 1.0 + (2.0 * leftRight)
        elif leftRight > 0.05:
            # Turning right
            driveRight *= 1.0 - (2.0 * leftRight)              
        # Set the motors to the new speeds
        TB.SetMotor1(driveLeft  * maxPower)
        TB.SetMotor2(driveRight * maxPower)

    else:
        print('Unhandled message: ', end ="")
        print(msg)

def RxLoop():
    global RPIcontrollerActive
    global UdpMavLink
    
    # Wait for the first heartbeat 
    print("Waiting for GCS heartbeat... ^C to end")
    msg = UdpMavLink.wait_heartbeat(blocking=True)           
    #   This sets the system and component ID of remote system for the link
    print("Heartbeat from system (system %u component %u)" % (msg.get_srcSystem(), msg.get_srcComponent()))
    while True:
        msg = UdpMavLink.recv_match(blocking=True)
        if msg.get_msgId() != mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT:
            handle_Messages(msg)

def HeartBeatLoop(): 
    global UdpMavLink
    global ledBatteryMode
    RPIcontrollerActive = True
    print ("Start sending heartbeat message... ^C to end")
    while RPIcontrollerActive:
        UdpMavLink.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GROUND_ROVER, mavutil.mavlink.MAV_AUTOPILOT_GENERIC, \
            mavutil.mavlink.MAV_MODE_MANUAL_DISARMED, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
        UdpMavLink.mav.sys_status_send(0,0,0,500,7400,330,50,0,0,0,0,0,0)
        # Change LEDs to purple to show motor faults
        if TB.GetDriveFault1() or TB.GetDriveFault2():
            if ledBatteryMode:
                TB.SetLedShowBattery(False)
                TB.SetLeds(1,0,1)
                ledBatteryMode = False
        else:
            if not ledBatteryMode:
                TB.SetLedShowBattery(True)
                ledBatteryMode = True

        time.sleep(0.9)

# Re-direct our output to standard error, we need to ignore standard out to hide some nasty print statements from pygame
sys.stdout = sys.stderr

# Setup the ThunderBorg
TB = ThunderBorg3.ThunderBorg()
#TB.i2cAddress = 0x15                  # Uncomment and change the value if you have changed the board address
TB.Init()
if not TB.foundChip:
    boards = ThunderBorg.ScanForThunderBorg()
    if len(boards) == 0:
        print ("No ThunderBorg found, check you are attached :")
    else:
        print ("No ThunderBorg at address %02X, but we did find boards:" % (TB.i2cAddress))
        for board in boards:
            print ("    %02X (%d)" % (board, board))
        print ("If you need to change the I�C address change the setup line so it is correct, e.g.")
        print ("TB.i2cAddress = 0x%02X" % (boards[0]))
    sys.exit()

# Ensure the communications failsafe has been enabled!
failsafe = False
for i in range(5):
    TB.SetCommsFailsafe(True)
    failsafe = TB.GetCommsFailsafe()
    if failsafe:
        break
    if not failsafe:
        print ("Board %02X failed to report in failsafe mode!' % (TB.i2cAddress)")
        sys.exit()

# Setup the power limits
if voltageOut > voltageIn:
    maxPower = 1.0
else:
    maxPower = voltageOut / float(voltageIn)

# Setup TB
TB.MotorsOff()
TB.SetLedShowBattery(False)
TB.SetLeds(0,0,1)
os.environ["SDL_VIDEODRIVER"] = "dummy" # Removes the need to have a GUI window
TB.SetLedShowBattery(True)
ledBatteryMode = True

driveLeft = 0.0
driveRight = 0.0
upDown = 0.0
leftRight = 0.0

# initialize threads
t_HeartBeatLoop = Thread(target=HeartBeatLoop)
t_RxLoop = Thread(target=RxLoop)

# Create the connection to the top-side computer as companion computer/autopilot
UdpMavLink = mavutil.mavlink_connection('udpout:192.168.178.25:14550', source_system=1, \
    source_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, loclAddr='0.0.0.0:14551')

# start the threads
try:
    t_HeartBeatLoop.start()
    t_RxLoop.start()

except KeyboardInterrupt:        
    # CTRL+C exit
    print ("\nUser shutdown")

finally:
    # Disable all drives
    TB.MotorsOff()
    TB.SetCommsFailsafe(False)
    TB.SetLedShowBattery(False)
    TB.SetLeds(0,0,0)
