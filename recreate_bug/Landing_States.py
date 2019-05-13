# Landing_States.py
# Tim Player
# 3 April 2019
# tplayer@hmc.edu

from Landing_State import Landing_State
from nav_helper_funcs import find_target, log_data, move_vel, move_pos
import global_params as gp
import dronekit


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
        the drone rises vertically to 10 meters.
        """

        x_m, y_m, x_pix, y_pix, frame = find_target(vs, vehicle)
        if x_m is not None:  # if a target was found, x_m won't be None.
            self.set_next_state('target_found')
            # record this sighting
            self.targ_sighting_loc = vehicle.location.global_relative_frame

        else:

            if self.targ_sighting_loc is None:
                # find current location
                loc_cur = vehicle.location.global_relative_frame

                # set desired location to have same lat, long but be 10 meters high
                loc_des = dronekit.LocationGlobalRelative(
                    loc_cur.lat, loc_cur.lon, 10)
            else:
                loc_des = self.targ_sighting_loc

            vehicle.simple_goto(loc_des)

        # DEBUG: REMOVE LOGGING
        #log_data(log_name, vehicle, x_m, y_m, x_pix, y_pix)
        # out.write(frame)


class Initial_Descent_State(Landing_State):
    """
    First, center the target in the sights. Then, descend. 
    """

    def set_next_state(self, event):
        if event == 'target_lost':
            self.next_state = Restart_State(self.targ_sighting_loc)

    def executeControl(self, vs, vehicle, out, log_name):
        """ 
        Control loop. Searches for target. If it finds the target, 
        it proceeds either horizontally or verticall until it is 
        directly above the target. 

        If no target is found, it switches to Lost_State.
        """

        x_m, y_m, x_pix, y_pix, frame = find_target(vs, vehicle)

        if x_m is not None:  # if a target was found, x_m will not be None.
            # record this sighting
            self.targ_sighting_loc = vehicle.location.global_relative_frame

            # proceed horizontally or vertically.
            if (x_m*x_m + y_m*y_m) > gp.descent_err*gp.descent_err:

                # go horizontally
                move_pos(vehicle, x_m, y_m, 0)
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

    def __init__(self):
        self.next_state = self
        self.time_that_copter_stopped = None
        print 'Processing current state:', str(self)

    def set_next_state(self, event):
        if event == 'target_lost':
            self.next_state = Lost_State(self.targ_sighting_loc)
        elif event == 'landed':
            self.next_state = Landed_State()

    def executeControl(self, vs, vehicle, out, log_name):
        """ 
        Control loop. Searches for target. If it finds the target, 
        it sends a LANDING_TARGET_ENCODE sequence.

        If no target is found, it switches to Lost_State.

        If it has been trying to descend for a long time, it turns off.
        """

        # if the vehicle has been stopped for 3 seconds, turn off.
        if abs(vehicle.velocity[2]) < gp.stopped_vel:
            if time_that_copter_stopped is None:
                time_that_copter_stopped = time.clock()
            elif time.clock() - time_that_copter_stopped > gp.terminate_time:
                self.set_next_state('landed')
                return

        x_m, y_m, x_pix, y_pix, frame = find_target(vs, vehicle)

        if x_m is not None:  # if a target was found, x_m will not be None.
            # record this sighting
            self.targ_sighting_loc = vehicle.location.global_relative_frame

            # calculate distance
            z = vehicle.location.global_relative_frame.alt
            dist = math.sqrt(x_m*x_m + y_m*y_m + z*z)

            send_land_message(x_pix, y_pix, dist)

        else:
            self.set_next_state('target_lost')

        log_data(gp.log_name, vehicle, x_m, y_m, x_pix, y_pix)
        out.write(frame)


class Landed_State(Landing_State):
    """
    Exit loop.
    """

    # this class implements no methods.
