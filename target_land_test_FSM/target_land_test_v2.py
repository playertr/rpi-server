#!/usr/bin/env python2

# target_land_test_v2.py
# Tim Player and Andrew Pham
# Questions? tplayer@hmc.edu, apham@hmc.edu
# Development version 17 May 2019

from global_params import og_horz_resolution, og_vert_resolution, horizontal_resolution, vertical_resolution
from nav_helper_funcs import getFPS, make_headers, terminate_flight

from Landing_States import Restart_State, Initial_Descent_State, Final_Descent_State, Landed_State

from collections import deque
from imutils.video import VideoStream
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
import time
import cv2  # version 2.4.13.6 https://docs.opencv.org/2.4.13.6/
import numpy as np
import datetime
from dronekit import VehicleMode, connect
from pymavlink import mavutil
import os

#####################################################################
# main method
#####################################################################


def main():

    vehicle = connect('/dev/serial0', wait_ready=False, baud=57600)
    # vehicle.parameters['PLND_EST_TYPE'] = 0  #  raw output
    # vehicle.flush()

    # time.sleep(0.5)
    #print "PLND_EST_TYPE: %s" % vehicle.parameters['PLND_EST_TYPE']

    # start input video stream
    vs = PiVideoStream(resolution=(
        og_horz_resolution, og_vert_resolution)).start()

    # wait for the camera to start
    time.sleep(0.5)

    # determine framerate
    fps = getFPS(vs, vehicle)

    # determine the current counter number of the log file
    # this assumes the file format is 'nameXXX.txt' where XXX is the number we want.
    def file_numbers(fpath):
        nums = []
        for filename in os.listdir(fpath):
            name, _ = os.path.splitext(filename)
            try:
                nums.append(int(name[-3:]))
            except:
                print("Log files do not follow nameXXX.txt convention.")
        return nums

    script_dir = os.path.dirname(os.path.realpath('__file__'))
    log_dir_relative = r"Log"
    log_dir = os.path.join(script_dir, log_dir_relative)
    numbers = file_numbers(log_dir)
    if not numbers:  # numbers is empty
        numbers = [0]
    count = max(numbers)
    count += 1
    count = format(count, '03')
    print(["File Number: ", count])

    # Define the codec and create VideoWriter object
    fourcc = cv2.cv.CV_FOURCC(*'DIVX')
    video_file = "Captures/camera_cap_" + count + ".avi"

    out = cv2.VideoWriter(video_file, fourcc, fps.fps(),
                          (horizontal_resolution, vertical_resolution))

    # Make Log headers
    log_name = 'Log/Bot_' + count + '.txt'
    make_headers(log_name)

    # control loop
    state = Restart_State()

    while(repr(state) != "Landed_State"):
        try:
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
