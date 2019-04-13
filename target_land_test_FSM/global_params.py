# global_params.py
# Tim Player
# 3 April 2019
import math

#####################################################################
# define constants
#####################################################################


# color settings
colorLower = (100, 50, 50)
colorHigher = (220, 255, 255)

# the smallest radius enclosing circle before it will register this
# as a target
min_radius = 10


descent_err = 2  # in Initial_Descent_State, the horizontal success radius
descent_vel = 2  # in Initial_Descent_State, the vertical descent rat

stopped_vel = 0.2  # 20 cm/s velocity means the vehicle has stopped
terminate_time = 3  # after being landed for 3 seconds, disarm

# camera
horizontal_fov = 62.2 * math.pi/180
vertical_fov = 48.8 * math.pi/180
og_horz_resolution = 640
og_vert_resolution = 480
horizontal_resolution = 320
vertical_resolution = 240
