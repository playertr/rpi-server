# nav_helper_funcs.py
# Tim Player
# 17 May 2019

from global_params import horizontal_resolution, colorLower, colorHigher, colorHigher, min_radius, horizontal_fov, vertical_fov, horizontal_resolution, vertical_resolution, og_horz_resolution, og_vert_resolution
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
import math
import pdb


def getFPS(vs, vehicle):
    """ Return an FPS object containing the framerate.

    Arguments:
        vs {PiVideoStream} -- The piCam input
        vehicle {Vehicle} -- dronekit Vehicle object (required by find_target)

    Returns:
        fps -- FPS object
    """

    fps = FPS().start()

    # Determine the frames per second of camera by running test loop 100 times
    while fps._numFrames < 100:
        # update the FPS counter and run through the image processing alg
        find_target(vs, vehicle)
        fps.update()

    # stop the timer and display FPS information
    fps.stop()
    print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

    return fps


def make_headers(file_name):

    f = open(file_name, 'a+')
    f.write('{0} {1:^1} {2:^1} {3:^1} {4:^1} {5:^1} {6:^1}\n'.format(
        'Lat', 'Lon', 'Alt', 'X-plat', 'Y-plat', 'X-px', "Y-px"))
    f.close()


def send_land_message(vehicle, x_rad, y_Rad, dist):
    """Sends MAV: LANDING_TARGET_ENCODE message to copter

    Arguments:
        x {int} -- target x centroid LOS angle in radians
        y {int} -- target y centroid LOS angle in radians
        dist {[type]} -- [description]
    """

    msg = vehicle.message_factory.landing_target_encode(
        0,       # time_boot_ms (not used)
        0,       # target num
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        x_rad,  # x-axis angular offset
        y_rad,  # y-axis angular offset
        dist,      # distance to target, in meters
        0, 0)     # size of target in radians
    vehicle.mode = VehicleMode("LAND")
    vehicle.send_mavlink(msg)
    vehicle.flush()


def goto(vehicle, lat, lon, z):
    print("going to position: {}".format([lat, lon, z]))
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b110111111000,  # Position typemask
        int(lat * 10**7),  # latitude
        int(lon * 10**7),  # longitude
        z,  # altitude in meters above home position
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    )
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.send_mavlink(msg)
    vehicle.flush()


def get_distance_meters(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def move_pos(vehicle, x, y, z):
    """
    Move, in the body frame, x meters forward, y meters right, and z 
    meters down

    Arguments:
        vehicle {Vehicle} -- dronekit Vehicle object
        x {float} -- meters to move forward
        y {float} --  meters to move right
        z {float} -- meters to move down
    """

    print("moving position: {}".format([x, y, z]))
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b110111111000,  # Position typemask
        x,  # dx forward (meters)
        y,  # dy right (meters)
        z,  # dz down (meters)
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    )
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.send_mavlink(msg)
    vehicle.flush()


def move_vel(vehicle, vx, vy, vz):
    """
    Move, in the body-oriented frame, x m/s forward, y m/s right, and z 
    m/s down

    Arguments:
        vehicle {Vehicle} -- dronekit Vehicle object
        vx {float} -- m/s forward
        vy {float} -- m/s right
        vz {float} -- m/s down
    """
    print("moving velocity: {}".format([vx, vy, vz]))
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b110111000111,  # Velocity typemask
        0,
        0,
        0,
        vx,  # dx forward (meters)
        vy,  # dy right (meters)
        vz,  # dz down (meters)
        0,
        0,
        0,
        0,  # yaw (0 is fwd)
        0)  # yaw rate rad/s
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.send_mavlink(msg)
    vehicle.flush()


def terminate_flight(vehicle):
    """
    Sends a MAV_CMD_DO_FLIGHTTERMINATION message. Immediately kills motors,
    causing the copter to a) fall from the air, or b) cease to macerate your
    fingers.

    May currently have the wrong number of parameters.

    https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FLIGHTTERMINATION

    Arguments:
        vehicle {Vehicle} -- dronekit Vehicle object
    """
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,  # command
        1,  # confirmation
        0,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        0,          # param 3, direction -1 ccw, 1 cw
        0,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used

    vehicle.send_mavlink(msg)
    vehicle.flush()


def log_data(file_name, vehicle, x_m, y_m, x_pix, y_pix):

    f = open(file_name, 'a+')

    # edit this line to have data logging of the data you care about
    data = [str(x) for x in [vehicle.location.global_relative_frame.lat,
                             vehicle.location.global_relative_frame.lon,
                             vehicle.location.global_relative_frame.alt,
                             x_m, y_m, x_pix, y_pix]]

    f.write(' '.join(data) + '\n')
    f.close()


def find_target(vs, vehicle):
    """
    Computes the lateral displacement in meters and pixels of the target from the drone. Writes the image to the output video stream.

    Arguments:
        vs {PiVideoStream} -- The piCam input
        vehicle {Vehicle} -- dronekit Vehicle object to find altitude

    Returns:
        (x_m, y_m, x_pix, y_pix, frame) -- lateral displacement in meters and pixels, plus video image
    """

    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 400 pixels
    frame = vs.read()

    frame = imutils.resize(
        frame, width=horizontal_resolution, height=vertical_resolution)

    blurred = cv2.GaussianBlur(frame, (3, 3), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "purple", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, colorLower, colorHigher)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)

    # find contours in the mask and initialize the current
    # (x, y) center of the target
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0]  # if imutils.is_cv2() else cnts[1]
    center = None

    # initialize return values to None
    (x_m, y_m, x_rad, y_rad) = (None, None, None, None)

    # only proceed if at least one contour was found
    if len(cnts) > 0:

        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)

        if M["m00"] != 0:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        else:
            center = (0, 0)

        # only proceed if the radius meets a minimum size
        if radius > min_radius:
            # print("Landing")
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            # cv2.circle(frame, (int(x), int(y)), int(radius),
            #    (255, 255, 0), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            z = vehicle.location.global_relative_frame.alt
            #print("Alt: ", z)
            m_per_pix_x = (2*z*math.tan(horizontal_fov/2)) / \
                horizontal_resolution
            m_per_pix_y = (2*z*math.tan(vertical_fov/2)) / \
                vertical_resolution

            x_m = (x-horizontal_resolution/2)*m_per_pix_x
            y_m = -(y-vertical_resolution/2)*m_per_pix_y
            dist = math.sqrt(x_m*x_m + y_m*y_m + z*z)
            x_rad = (x-horizontal_resolution/2) * \
                horizontal_fov/horizontal_resolution
            y_rad = (y-vertical_resolution/2) * \
                vertical_fov/vertical_resolution

    return (x_m, y_m, x_rad, y_rad, frame)
