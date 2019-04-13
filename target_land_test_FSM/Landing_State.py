# State.py
# @author Tim Player, based on Karn Saheb's state machine implementation
# tplayer@hmc.edu
# 29 March 2019


class Landing_State(object):
    """
    We define a state object which provides some utility functions for the
    individual states within the state machine.
    """

    def __init__(self, targ_sighting_loc=None):
        self.next_state = self
        self.targ_sightin_loc = targ_sighting_loc  # location of latest target sighting
        print 'Processing current state:', str(self)

    def transition(self):
        """
        Transitions to the next state based on an event.
        """
        return self.next_state

    def set_next_state(self, event):
        """
        Sets the next_state field to be a different LandingState
        object

        Arguments:
            event {string} -- flag such as 'target_found'

        """
        raise NotImplementedError

    def executeControl(self, vs, vehicle, out):
        """
        Inherited (base) control loop for LandingState object

        Arguments:
            vs {PiVideoStream} -- the videostream object from the cam
            vehicle {Vehicle} -- the Dronekit Vehical object
            out {cv2.VideoWriter} -- the output vidoestream
        """
        raise NotImplementedError

    def __repr__(self):
        """
        Leverages the __str__ method to describe the State.
        """
        return self.__str__()

    def __str__(self):
        """
        Returns the name of the State.
        """
        return self.__class__.__name__
