#!/usr/bin/env python2

# target_land_test_v2.py
# Tim Player and Andrew Pham
# Questions? tplayer@hmc.edu, apham@hmc.edu
# Development version 29 March 2019

import global_params
import nav_helper_funcs
from Landing_States import Initial_Descent_State, \
    Final_Descent_State, Restart_State, Landed_State

#####################################################################
# main method
#####################################################################


def main():
    # start input video stream
    vs = PiVideoStream(resolution=(
        og_horz_resolution, og_vert_resolution)).start()

    # label this capture
    start_time = datetime.datetime.now().replace(microsecond=0).strftime(
        '%y-%m-%d %H.%M.%S')

    # Define the codec and create VideoWriter object
    fourcc = cv2.cv.CV_FOURCC(*'DIVX')
    video_file = "Captures/camera_cap_" + \
        start_time + ".avi"
    out = cv2.VideoWriter(video_file, fourcc, fps.fps(),
                          (horizontal_resolution, vertical_resolution))

    vehicle = connect('/dev/serial0', wait_ready=False, baud=57600)

    # Make Log headers
    log_name = 'Log/Bot_' + start_time + '.txt'
    make_headers(log_name)

    # control loop
    state = Searching_State()

    while(repr(state) != "Landed_State"):
        try:
            state.executeControl(vs, vehicle, out)
            state = state.transition()

        except (KeyboardInterrupt, SystemExit):
            break

    terminate_flight(vehicle)


if __name__ == "__main__":
    main()
