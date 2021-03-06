#!/usr/bin/env python2

# target_land_test_v2.py
# Tim Player and Andrew Pham
# Questions? tplayer@hmc.edu, apham@hmc.edu
# Development version 29 March 2019

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

#####################################################################
# main method
#####################################################################


def main():

    vehicle = connect('/dev/serial0', wait_ready=False, baud=57600)

    # start input video stream
    vs = PiVideoStream(resolution=(
        og_horz_resolution, og_vert_resolution)).start()

    # wait for the camera to start
    time.sleep(0.5)

    # determine framerate
    #fps = getFPS(vs, vehicle)
    fps = 30  # DEBUG
    # label this capture
    start_time = datetime.datetime.now().replace(microsecond=0).strftime(
        '%y-%m-%d %H.%M.%S')

    # Define the codec and create VideoWriter object
    fourcc = cv2.cv.CV_FOURCC(*'DIVX')
    video_file = "Captures/camera_cap_" + start_time + ".avi"

    # out = cv2.VideoWriter(video_file, fourcc, fps.fps(),
    #                       (horizontal_resolution, vertical_resolution))
    out = cv2.VideoWriter(video_file, fourcc, 30,
                          (horizontal_resolution, vertical_resolution))

    # Make Log headers
    log_name = 'Log/Bot_' + start_time + '.txt'
    make_headers(log_name)

    # control loop
    state = Restart_State()

    # track memory leaks
    from pympler import tracker, summary, muppy
    #tr = tracker.SummaryTracker()

    while(repr(state) != "Landed_State"):
        try:
            # tr.print_diff()
            #sum1 = summary.summarize(muppy.get_objects())
            # summary.print_(sum1)

            ###########################
            # Print the current frame
            frame = vs.read()
            cv2.imshow('frame', frame)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
            ###########################

            #state.executeControl(vs, vehicle, out, log_name)
            # DEBUG: DO NOT TRANSITION!
            #state = state.transition()

        except (KeyboardInterrupt, SystemExit):
            break

    terminate_flight(vehicle)
    out.release()
    vehicle.close()
    cv2.destroyAllWindows()
    vs.stop()


if __name__ == "__main__":
    main()
