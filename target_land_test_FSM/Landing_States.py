# Landing_States.py
# Tim Player
# 27 May 2019
# tplayer@hmc.edu

from Landing_State import Landing_State
from nav_helper_funcs import find_target, log_data, move_vel, move_pos, send_land_message, goto, get_distance_meters
import global_params as gp
import dronekit
import time
import math

"""
    This file defines several children of the Landing_State superclass. Currently, Restart_State, and Final_Descent_State are used.
"""


class Restart_State(Landing_State):
    """
    Go to the location where the target was last sighted.
    """

    def __init__(self, targ_sighting_loc=None):
        super(Restart_State, self).__init__(targ_sighting_loc)
        self.command_issued = False
        self.destination_reached = False
        self.loc_des = None  # set in executeControl

    def set_next_state(self, event):
        # to enter Initial_Descent_State, the copter must be near its restart point and it must see the target.
        if event == 'target_found' and self.destination_reached:
            self.next_state = Final_Descent_State(self.targ_sighting_loc)

    def executeControl(self, vs, vehicle, out, log_name):
        """ 
        Control loop. Searches for target. If it finds the target, next_state is set to Final_Descent_State. Otherwise, the drone resets to a) targ_sighting_loc or b) its current position, with a desired altitude of gp.restart_height.
        """

        # Search for the target
        x_m, y_m, x_rad, y_rad, frame = find_target(vs, vehicle)

        # If target found, record the sighting event
        if x_m is not None:
            self.set_next_state('target_found')
            self.targ_sighting_loc = vehicle.location.global_relative_frame

        # If a goto command has not already been issued, issue a goto command.
        if not self.command_issued:
            if self.targ_sighting_loc is None:
                self.loc_des = vehicle.location.global_relative_frame
            else:
                self.loc_des = self.targ_sighting_loc
            goto(vehicle, self.loc_des.lat, self.loc_des.lon, gp.restart_height)
            self.command_issued = True

        # See whether the reset destination has been reached
        err = get_distance_meters(
            vehicle.location.global_relative_frame, self.loc_des)

        if err < gp.restart_err:
            self.destination_reached = True

        log_data(log_name, vehicle, x_m, y_m, x_rad, y_rad)
        out.write(frame)


class Final_Descent_State(Landing_State):
    """
    Initiate and complete a Mav LANDING_TARGET_ENCODE sequence.
    """

    def __init__(self, targ_sighting_loc=None):
        super(Final_Descent_State, self).__init__(targ_sighting_loc)
        self.time_that_copter_stopped = None
        self.last_sight_time = time.clock()

    def set_next_state(self, event):
        if event == 'target_lost' and (time.clock() - self.last_sight_time > gp.time_till_lost):
            self.next_state = Restart_State(self.targ_sighting_loc)

    def executeControl(self, vs, vehicle, out, log_name):
        """ 
        Control loop. Searches for target. If it finds the target, 
        it sends a LANDING_TARGET_ENCODE sequence.

        If no target is found for long enough, it switches to Lost_State.
        """

        x_m, y_m, x_rad, y_rad, frame = find_target(vs, vehicle)

        if x_m is not None:  # if a target was found, x_m will not be None.
            # record this sighting
            self.targ_sighting_loc = vehicle.location.global_relative_frame
            self.last_sight_time = time.clock()
            # calculate distance
            z = vehicle.location.global_relative_frame.alt
            dist = math.sqrt(x_m*x_m + y_m*y_m + z*z)
            send_land_message(vehicle, x_rad, y_rad, dist)

        else:
            self.set_next_state('target_lost')

        log_data(log_name, vehicle, x_m, y_m, x_rad, y_rad)
        out.write(frame)

##################### DEPRECATED CODE ################################

# class Initial_Descent_State(Landing_State):
#     """
#     \deprecated
#     DEPRECATED DUE TO ERRATIC MOV_POS BEHAVIOR.
#     First, center the target in the sights. Then, descend.
#     """

#     def __init__(self, targ_sighting_loc=None):
#         super(Initial_Descent_State, self).__init__(targ_sighting_loc)
#         self.last_sight_time = time.clock()

#     def __init__(self, targ_sighting_loc=None):
#         super(Initial_Descent_State, self).__init__(targ_sighting_loc)
#         self.last_sight_time = time.clock()

#     def set_next_state(self, event):
#         if event == 'target_lost' and (time.clock() - self.last_sight_time > gp.time_till_lost):
#             self.next_state = Restart_State(self.targ_sighting_loc)
#         elif event == 'low_altitude':
#             self.next_state = Final_Descent_State(self.targ_sighting_loc)

#     def executeControl(self, vs, vehicle, out, log_name):
#         """
#         Control loop. Searches for target. If it finds the target,
#         it proceeds either horizontally or vertically until it is
#         directly above the target.

#         If no target is found, it switches to Lost_State.
#         """
#         z = vehicle.location.global_relative_frame.alt

#         if z < gp.final_descent_alt:  # if height < 4 meters
#             self.set_next_state('low_altitude')

#         x_m, y_m, x_rad, y_rad, frame = find_target(vs, vehicle)

#         if x_m is not None:  # if a target was found, x_m will not be None.
#             # record this sighting
#             self.targ_sighting_loc = vehicle.location.global_relative_frame
#             self.last_sight_time = time.clock()

#             # proceed horizontally or vertically.
#             if (x_m*x_m + y_m*y_m) > gp.descent_err*gp.descent_err:
#                 # go horizontally
#                 move_pos(vehicle, -x_m, -y_m, 0)
#             else:
#                 # go vertically
#                 move_vel(vehicle, 0, 0, gp.descent_vel)

#         else:
#             self.set_next_state('target_lost')

#         log_data(log_name, vehicle, x_m, y_m, x_rad, y_rad)
#         out.write(frame)

# class Landed_State(Landing_State):
#     """
#     Exit loop.
#     """

#     # this class implements no methods.
