from __future__ import print_function
import numpy as np
import rospy
from porj5.srv import get_state
import sucker 
import control_test
import time
from tf.transformations import quaternion_from_euler
from math import pi, cos, sin
# init
def get_state_client():
    rospy.wait_for_service('get_state')
    
    state_list = rospy.ServiceProxy('get_state', get_state)
    resp = state_list(int(1))
    return resp.state_list
# paras
hand_eye_matrix = np.array([[-9.61033002e-01  ,8.92961762e-03  ,3.58493833e-02  ,5.39771005e-01],
 [ 1.43865111e-02  ,9.65756542e-01 ,-3.79220285e-02  ,3.31974356e-01],
 [-7.09848298e-03 ,-3.11624787e-02 ,-8.85621808e-01  ,9.65180690e-01],
 [-2.45517492e-15  ,2.22044605e-16 ,-5.55111512e-17  ,1.00000000e+00]])
'''
hand_eye_matrix = np.array([[-9.86048652e-01  ,5.30317719e-02 ,-2.18483014e-01  ,7.44736034e-01],
 [ 2.25464562e-02  ,9.57351699e-01  ,2.55816896e-02  ,2.58878394e-01],
 [ 2.57828147e-02 ,-6.70507274e-02 ,-7.50075913e-01  ,8.55685708e-01],
 [ 1.99283667e-15 ,-2.22044605e-16 ,-6.66133815e-16  ,1.00000000e+00]])'''
# panda
pc = control_test.my_panda()
x_origin = 0.571507
y_origin = -0.28287
z_high = 0.2599
z_low = 0.06
q = quaternion_from_euler(pi, 0, 0)

panda_origin = pc.set_goal([x_origin, y_origin, z_high + 0.1], q)
panda_origin = pc.set_goal([x_origin, y_origin, z_high], q)
# sucker
sucker = sucker.sucker()

# home
pc.move_arm(panda_origin)
# main loop

while True:

    resp = get_state_client()

    if sum(resp) == 0:
        break

    for index in [0, 1, 3, 2]:
        if resp[index*6] !=0:
            break
    
    loc_x, loc_y, goal_x, goal_y, piece_ang, base_ang = resp[6*index:6*index + 6]
    loc_x_p = loc_x + 426
    loc_y_p = loc_y + 30
    goal_x_p = goal_x + 426
    goal_y_p = goal_y + 30
    loc_x = np.multiply(loc_x_p-624.79,1.05/931.69)
    loc_y = np.multiply(loc_y_p-360.52,1.05/931.46)
    goal_x = np.multiply(goal_x_p-624.79,1.05/931.69)
    goal_y = np.multiply(goal_y_p-360.52,1.05/931.46)
    # pick
    pick_location = np.array([loc_x, loc_y, 1.05, 1])
    pick_location = hand_eye_matrix.dot(pick_location.T)
    # place 
    place_location = np.array([goal_x, goal_y, 1.05, 1])
    place_location = hand_eye_matrix.dot(place_location.T)
    # q
    piece_q = quaternion_from_euler(pi, 0, -piece_ang*pi/180)
    base_q = quaternion_from_euler(pi, 0, -base_ang*pi/180)
    if index == 0:

        # place on mid point of base
        # move -x
        # move -y 
        goal = pc.set_goal([pick_location[0], pick_location[1]-0.05, z_high], piece_q)
        pc.move_arm(goal) # move above to piece 

        goal = pc.set_goal([pick_location[0], pick_location[1]-0.05, z_low], piece_q)
        pc.move_arm(goal) # move to piece

        sucker.suck() # pick 

        goal = pc.set_goal([pick_location[0], pick_location[1]-0.05, z_high], piece_q)
        pc.move_arm(goal) # move to piece

        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_high], base_q)
        pc.move_arm(goal) # move to piece

        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_low + 0.007 ], base_q)
        pc.move_arm(goal) # move to piece
        
        goal_x_p = goal_x_p - (25)*cos(base_ang*pi/180)
        goal_y_p = goal_y_p - (25)*sin(base_ang*pi/180)
        goal_x = np.multiply(goal_x_p-624.79,1.05/931.69)
        goal_y = np.multiply(goal_y_p-360.52,1.05/931.46)
        place_location = np.array([goal_x, goal_y, 1.05, 1])
        place_location = hand_eye_matrix.dot(place_location.T)
        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_low + 0.007 ], base_q)
        pc.move_arm(goal)

        goal_x_p = goal_x_p - (25)*cos((base_ang + 90)*pi/180)
        goal_y_p = goal_y_p - (25)*sin((base_ang + 90)*pi/180)
        goal_x = np.multiply(goal_x_p-624.79,1.05/931.69)
        goal_y = np.multiply(goal_y_p-360.52,1.05/931.46)
        place_location = np.array([goal_x, goal_y, 1.05, 1])
        place_location = hand_eye_matrix.dot(place_location.T)
        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_low + 0.007 ], base_q)
        pc.move_arm(goal)

        sucker.loose()

        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_high + 0.007 ], base_q)
        pc.move_arm(goal)
        pc.move_arm(panda_origin)


    elif index == 1:
        piece_q = quaternion_from_euler(pi, 0, (-piece_ang + 90)*pi/180)
        goal = pc.set_goal([pick_location[0], pick_location[1]-0.05, z_high], piece_q)
        pc.move_arm(goal) # move above to piece 

        goal = pc.set_goal([pick_location[0], pick_location[1]-0.05, z_low], piece_q)
        pc.move_arm(goal) # move to piece

        sucker.suck() # pick 

        goal = pc.set_goal([pick_location[0], pick_location[1]-0.05, z_high], piece_q)
        pc.move_arm(goal)

        near_q = quaternion_from_euler(pi, (base_ang)*pi/180, -10*pi/180, axes='rxzy')

        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_high], near_q)
        pc.move_arm(goal)

        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_low + 0.008], near_q)
        pc.move_arm(goal) # near

        goal_x_p = goal_x_p - (20)*cos((base_ang + 90)*pi/180)
        goal_y_p = goal_y_p - (20)*sin((base_ang + 90)*pi/180)
        goal_x = np.multiply(goal_x_p-624.79,1.05/931.69)
        goal_y = np.multiply(goal_y_p-360.52,1.05/931.46)
        place_location = np.array([goal_x, goal_y, 1.05, 1])
        place_location = hand_eye_matrix.dot(place_location.T)
        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_low + 0.008], near_q)
        pc.move_arm(goal)

        goal_x_p = goal_x_p + (20)*cos((base_ang + 180)*pi/180)
        goal_y_p = goal_y_p + (20)*sin((base_ang + 180)*pi/180)
        goal_x = np.multiply(goal_x_p-624.79,1.05/931.69)
        goal_y = np.multiply(goal_y_p-360.52,1.05/931.46)
        place_location = np.array([goal_x, goal_y, 1.05, 1])
        place_location = hand_eye_matrix.dot(place_location.T)
        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_low + 0.008], near_q)
        pc.move_arm(goal)

        sucker.loose()

        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_high], base_q)
        pc.move_arm(goal)
        pc.move_arm(panda_origin)
        # place 
        # move -x
        # move -y
    elif index == 2:
        piece_q = quaternion_from_euler(pi, 0, (-piece_ang + 45 + 90)*pi/180)
        goal = pc.set_goal([pick_location[0], pick_location[1]-0.05, z_high], piece_q)
        pc.move_arm(goal) # move above to piece 

        goal = pc.set_goal([pick_location[0], pick_location[1]-0.05, z_low], piece_q)
        pc.move_arm(goal) # move to piece

        sucker.suck() # pick 

        goal = pc.set_goal([pick_location[0], pick_location[1]-0.05, z_high], piece_q)
        pc.move_arm(goal)

        

        near_q = quaternion_from_euler(pi, (base_ang + 45 + 90)*pi/180, -15*pi/180, axes='rxzy')

        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_high], near_q)
        pc.move_arm(goal)

        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_low + 0.01], near_q)
        pc.move_arm(goal)

        goal_x_p = goal_x_p + (25)*cos((base_ang)*pi/180)
        goal_y_p = goal_y_p + (25)*sin((base_ang)*pi/180)
        goal_x = np.multiply(goal_x_p-624.79,1.05/931.69)
        goal_y = np.multiply(goal_y_p-360.52,1.05/931.46)
        place_location = np.array([goal_x, goal_y, 1.05, 1])
        place_location = hand_eye_matrix.dot(place_location.T)
        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_low + 0.008], near_q)
        pc.move_arm(goal)

        goal_x_p = goal_x_p + (25)*cos((base_ang - 90)*pi/180)
        goal_y_p = goal_y_p + (25)*sin((base_ang - 90)*pi/180)
        goal_x = np.multiply(goal_x_p-624.79,1.05/931.69)
        goal_y = np.multiply(goal_y_p-360.52,1.05/931.46)
        place_location = np.array([goal_x, goal_y, 1.05, 1])
        place_location = hand_eye_matrix.dot(place_location.T)
        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_low + 0.008], near_q)
        pc.move_arm(goal)

        sucker.loose()

        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_high], base_q)
        pc.move_arm(goal)
        pc.move_arm(panda_origin)
        
        # place
        # move -x
        # move -y
    
    elif index == 3:
        piece_q = quaternion_from_euler(pi, 0, (-piece_ang + 90)*pi/180)
        goal = pc.set_goal([pick_location[0], pick_location[1]-0.05, z_high], piece_q)
        pc.move_arm(goal) # move above to piece 

        goal = pc.set_goal([pick_location[0], pick_location[1]-0.05, z_low], piece_q)
        pc.move_arm(goal) # move to piece

        sucker.suck() # pick 

        goal = pc.set_goal([pick_location[0], pick_location[1]-0.05, z_high], piece_q)
        pc.move_arm(goal)

        near_q = quaternion_from_euler(pi, (base_ang + 90)*pi/180, -10*pi/180, axes='rxzy')
        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_high], near_q)
        pc.move_arm(goal)
        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_low + 0.008], near_q)
        pc.move_arm(goal) # near

        goal_x_p = goal_x_p + (30)*cos((base_ang )*pi/180)
        goal_y_p = goal_y_p + (30)*sin((base_ang )*pi/180)
        goal_x = np.multiply(goal_x_p-624.79,1.05/931.69)
        goal_y = np.multiply(goal_y_p-360.52,1.05/931.46)
        place_location = np.array([goal_x, goal_y, 1.05, 1])
        place_location = hand_eye_matrix.dot(place_location.T)
        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_low + 0.008], near_q)
        pc.move_arm(goal)

        goal_x_p = goal_x_p + (15)*cos((base_ang + 270)*pi/180)
        goal_y_p = goal_y_p + (15)*sin((base_ang + 270)*pi/180)
        goal_x = np.multiply(goal_x_p-624.79,1.05/931.69)
        goal_y = np.multiply(goal_y_p-360.52,1.05/931.46)
        place_location = np.array([goal_x, goal_y, 1.05, 1])
        place_location = hand_eye_matrix.dot(place_location.T)
        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_low + 0.008], near_q)
        pc.move_arm(goal)

        sucker.loose()

        goal = pc.set_goal([place_location[0], place_location[1]-0.05, z_high], base_q)
        pc.move_arm(goal)
        pc.move_arm(panda_origin)



        
        # place 
        # move -x-y
    '''
    loc_x = loc_x + 426
    loc_y = loc_y + 30
    goal_x = goal_x + 426
    goal_y = goal_y + 30
    loc_x = np.multiply(loc_x-624.79,1.05/931.69)
    loc_y = np.multiply(loc_y-360.52,1.05/931.46)
    
    # pick
    location = np.array([loc_x, loc_y, 1.05, 1])
    location = hand_eye_matrix.dot(location.T)

    goal = pc.set_goal([location[0], location[1]-0.05, z_high], q)
    pc.move_arm(goal) # move above to piece 


    goal = pc.set_goal([location[0], location[1]-0.05, z_low], q)
    pc.move_arm(goal) # move to piece 

    sucker.suck() # pick 


    goal = pc.set_goal([location[0], location[1]-0.05, z_high], q)

    pc.move_arm(goal) # move above to piece 

    goal_x = np.multiply(goal_x-624.79,1.05/931.69)
    goal_y = np.multiply(goal_y-360.52,1.05/931.46)
    location = np.array([goal_x, goal_y, 1.05, 1])
    location = hand_eye_matrix.dot(location.T)
    q_ang = quaternion_from_euler(pi, 0, pi*ang/180)

    goal = pc.set_goal([location[0], location[1]-0.05, z_high], q_ang)
    pc.move_arm(goal) # move above to base

    goal = pc.set_goal([location[0], location[1]-0.04, z_low + 0.01], q_ang)
    pc.move_arm(goal) # move to base

    sucker.loose()

    goal = pc.set_goal([location[0], location[1]-0.04, z_high], q)
    pc.move_arm(goal) # move to base


    pc.move_arm(panda_origin) # to origin 
    '''