# Landing_States.py
# Tim Player
# 14 May 2019
# tplayer@hmc.edu

from Landing_State import Landing_State
from nav_helper_funcs import find_target, log_data,goto,  move_vel, move_pos, send_land_message
import global_params as gp
import dronekit
import time
import math

class Restart_State(Landing_State):
    """
    Go to the location where the target was last sighted.
    """

    def set_next_state(self, event):
        if event == 'target_found':
            self.next_state = Initial_Descent_State(self.targ_sighting_loc)

    def executeControl(self, vs, vehicle, out, log_name):
        """ 
        Control loop. Searches for target. If it finds the target, 
        next_state is set to Initial_Descent_State. Otherwise,
        the drone rises vertically to 7 meters.
        """

        x_m, y_m, x_pix, y_pix, frame = find_target(vs, vehicle)
        
        z = vehicle.location.global_relative_frame.alt
        # target must be found and altitude must be high
        if x_m is not None and z > 5:  # if a target was found, x_m won't be None.
            self.set_next_state('target_found')
            # record this sighting
            self.targ_sighting_loc = vehicle.location.global_relative_frame

        else:

            if self.targ_sighting_loc is None:
                # find current location
                loc_cur = vehicle.location.global_relative_frame

                # set desired location to have same lat, long but be 7 meters high (gp.restart_height)
                loc_des = dronekit.LocationGlobalRelative(
                    loc_cur.lat, loc_cur.lon, gp.restart_height)

            else:
                loc_des = self.targ_sighting_loc

            goto(vehicle, loc_des.lat, loc_des.lon, gp.restart_height)

        log_data(log_name, vehicle, x_m, y_m, x_pix, y_pix)
        out.write(frame)


class Initial_Descent_State(Landing_State):
    """
    First, center the target in the sights. Then, descend. 
    """

    def set_next_state(self, event):
        if event == 'target_lost':
            self.next_state = Restart_State(self.targ_sighting_loc)
        elif event == 'low_altitude':
            self.next_state = Final_Descent_State(self.targ_sighting_loc)

    def executeControl(self, vs, vehicle, out, log_name):
        """ 
        Control loop. Searches for target. If it finds the target, 
        it proceeds either horizontally or verticall until it is 
        directly above the target. 

        If no target is found, it switches to Lost_State.
        """
        z = vehicle.location.global_relative_frame.alt

        if z < gp.final_descent_alt:  # if height < 4 meters

            self.set_next_state('low_altitude')

        x_m, y_m, x_pix, y_pix, frame = find_target(vs, vehicle)

        if x_m is not None:  # if a target was found, x_m will not be None.
            # record this sighting
            self.targ_sighting_loc = vehicle.location.global_relative_frame

            # proceed horizontally or vertically.
            if (x_m*x_m + y_m*y_m) > gp.descent_err*gp.descent_err:

                # go horizontally
                move_pos(vehicle, -x_m, -y_m, 0)
            else:
                # go vertically
                move_vel(vehicle, 0, 0, gp.descent_vel)

        else:
            self.set_next_state('target_lost')

        log_data(log_name, vehicle, x_m, y_m, x_pix, y_pix)
        out.write(frame)


class Final_Descent_State(Landing_State):
    """
    Initiate, complete Mav LANDING_TARGET_ENCODE sequence.
    """

    def __init__(self, targ_sighting_loc=None):
        self.next_state = self
        self.time_that_copter_stopped = None
        self.targ_sighting_loc = targ_sighting_loc
        print('Processing current state:', str(self))

    def set_next_state(self, event):
        if event == 'target_lost':
            self.next_state = Restart_State(self.targ_sighting_loc)
        elif event == 'landed':
            self.next_state = Landed_State()

    def executeControl(self, vs, vehicle, out, log_name):
        """ 
        Control loop. Searches for target. If it finds the target, 
        it sends a LANDING_TARGET_ENCODE sequence.

        If no target is found, it switches to Lost_State.

        If it has been trying to descend for a long time, it turns off.
        """

        # this turns off too easy (even when in air!)
        # # if the vehicle has been stopped for 3 seconds, turn off.
        # if abs(vehicle.velocity[2]) < gp.stopped_vel:
        #     if self.time_that_copter_stopped is None:
        #         self.time_that_copter_stopped = time.clock()
        #     elif time.clock() - self.time_that_copter_stopped > gp.terminate_time:
        #         self.set_next_state('landed')
        #         return

        x_m, y_m, x_pix, y_pix, frame = find_target(vs, vehicle)

        if x_m is not None:  # if a target was found, x_m will not be None.
            # record this sighting
            self.targ_sighting_loc = vehicle.location.global_relative_frame

            # calculate distance
            z = vehicle.location.global_relative_frame.alt
            dist = math.sqrt(x_m*x_m + y_m*y_m + z*z)

            send_land_message(vehicle, x_pix, y_pix, dist)

        else:
            self.set_next_state('target_lost')

        log_data(log_name, vehicle, x_m, y_m, x_pix, y_pix)
        out.write(frame)


class Landed_State(Landing_State):
    """
    Exit loop.
    """

    # this class implements no methods.
