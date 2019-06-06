import control_test
import time
from tf.transformations import quaternion_from_euler
from math import pi

pc = control_test.my_panda()

x_origin = 0.571507
y_origin = -0.28287
z_high = 0.2599
z_low = 0.065
i = 0

    
goal = pc.set_goal([x_origin, y_origin, z_high], quaternion_from_euler(pi, 0, 0))
pc.move_arm(goal)
goal = pc.set_goal([x_origin, y_origin, z_high], quaternion_from_euler(pi, 0, pi*30/180))
pc.move_arm(goal)
#goal = pc.set_goal([x_origin, y_origin, z_high], quaternion_from_euler(pi, 0, -pi))
#pc.move_arm(goal)
