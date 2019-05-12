#!/usr/bin/env python2

# tlt_sim.py
# Tim Player and Andrew Pham
# Questions? tplayer@hmc.edu, apham@hmc.edu
# Development version 19 April 2019

##########################################################################################################################################################################################################
# NOTE s For TIM:
# remember to make set_next_state a self function!
# add a move_horz fnc and import statement
# self.set_next_state() instead of self.next_state
# line 60 calls Lost_State, which hasn't been defined.


from global_params import og_horz_resolution, og_vert_resolution, horizontal_resolution, vertical_resolution
from nav_helper_funcs import getFPS, make_headers

from Landing_States import Restart_State, Initial_Descent_State, Final_Descent_State, Landed_State

from collections import deque
from imutils.video import VideoStream
#from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
#from picamera.array import PiRGBArray
#from picamera import PiCamera
import argparse
import imutils
import time
import cv2  # version 2.4.13.6 https://docs.opencv.org/2.4.13.6/
import numpy as np
import datetime
from dronekit import VehicleMode, connect
from pymavlink import mavutil
import os
import pdb

#####################################################################
# arm_and_takeoff
#####################################################################


def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

#####################################################################
# main method
#####################################################################


def main():
    #vehicle = connect('/dev/serial0', wait_ready=False, baud=57600)
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
    vehicle = connect(connection_string, wait_ready=False, )
    # NOTE these are *very inappropriate settings*
    # to make on a real vehicle. They are leveraged
    # exclusively for simulation. Take heed!!!
    vehicle.parameters['FS_GCS_ENABLE'] = 0
    vehicle.parameters['FS_EKF_THRESH'] = 100

    arm_and_takeoff(vehicle, 10)

    # navigate to video  and log files.
    filestring = r"../../../Landing_Tests/7-18-18 landing vids"
    fileDir = os.path.dirname(os.path.realpath('__file__'))
    dataDir = os.path.join(fileDir, filestring)
    dataDir = os.path.abspath(os.path.realpath(dataDir))
    fileNames = os.listdir(dataDir)

    vidNames = [f for f in fileNames if f.endswith('.avi')]
    vidfilepath = os.path.join(dataDir, vidNames[1])  # select fourth video

    # start input video stream
    #vs = PiVideoStream(resolution=(og_horz_resolution, og_vert_resolution)).start()

    # vs.read() returns a tuple. I want the second item, Item 1. I want to dynamically replace every instance of vs.read() with vs.read()[1]. How do I override that function? With this class.
    class vidstream_wrapper():
        def __init__(self, vs):
            self.vs = vs

        def read(self):
            return self.vs.read()[1]

    vs = vidstream_wrapper(cv2.VideoCapture(vidfilepath))

    # wait for the camera to start
    time.sleep(0.5)

    fps = getFPS(vs, vehicle)

    # label this capture
    start_time = datetime.datetime.now().replace(microsecond=0).strftime(
        '%y-%m-%d %H.%M.%S')

    # Define the codec and create VideoWriter object
    fourcc = cv2.cv.CV_FOURCC(*'DIVX')
    video_file = "Captures/camera_cap_" + start_time + ".avi"

    out = cv2.VideoWriter(video_file, fourcc, fps.fps(),
                          (horizontal_resolution, vertical_resolution))

    # Make Log headers
    log_name = 'Log/Bot_' + start_time + '.txt'
    make_headers(log_name)
    # control loop
    state = Restart_State()

    while(repr(state) != "Landed_State"):
        try:
            # show frame. Note: this destroys a frame.
            frame = vs.read()
            cv2.imshow('frame', frame)

            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

            state.executeControl(vs, vehicle, out, log_name)
            state = state.transition()

        except (KeyboardInterrupt, SystemExit):
            break

    terminate_flight(vehicle)

    out.release()
    vehicle.close()
    cv2.destroyAllWindows()
    vs.stop()


if __name__ == "__main__":
    main()
