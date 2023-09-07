#!/usr/bin/python
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
import sys
from hsrb_interface import geometry
from geometry_msgs.msg import WrenchStamped
import time

# Move timeout[s]
_MOVE_TIMEOUT=60.0
# Grasp force[N]
_GRASP_FORCE=0.01
# TF name of the bottle
#real robot aruco marker
#_BOTTLE_TF='ar_marker/6'
# simulation aruco marker
_BOTTLE_TF='ar_marker/4000'
# TF name of the gripper
_HAND_TF='hand_palm_link'

# Preparation for using the robot functions
robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
gripper = robot.get('gripper')
tts = robot.get('default_tts')

# Posture that 0.02[m] front and rotate -1.57 around z-axis of the bottle maker
bottle_to_hand = geometry.pose(z=-0.02, ek=-1.57)

# Posture to move the hand 0.1[m] up
hand_up = geometry.pose(x=0.1)

#Right
#R = geometry.pose(y=0.4, x=0.2)

#Left
#L = geometry.pose(y=-0.55, x=0.2)
L = geometry.pose(y=-0.4, x=0.2)
#L = geometry.pose(y=-0.15, x=0.2)


# Posture to move the hand 0.5[m] back
hand_back = geometry.pose(z=-0.5)

# Location of the sofa neutral go
#sofa_pos = (1.2, 0.4, 1.57)

ft_sensor_topic = '/hsrb/wrist_wrench/raw'

wrench_force_z = 0.0
def wrist_wrench_cllbck(data):
    global wrench_force_z
    wrench_force_z = data.wrench.force.z

if __name__=='__main__':

    # Greet
    rospy.sleep(5.0)
    tts.say('Hi. My name is HSR. I will pick the PET bottle at side of the sofa.')
    rospy.sleep(2.0)
#    rosservice call /marker/start_recognition "{}"
#    time.sleep(5)
    try:
        wrench_sub = rospy.Subscriber('/hsrb/wrist_wrench/raw', WrenchStamped, wrist_wrench_cllbck)
        # Transit to initial grasping posture
        gripper.command(0)
        gripper.command(1.2)
        #Position for smooth transition
        omni_base.go_rel(-0.1, 0.0, 0.0, 10.0)
        whole_body.move_to_neutral()
        #Favouring hand movement more than omni_base
        whole_body.linear_weight = 100.0
        # Look at the hand
        whole_body.looking_hand_constraint = True
        # gripper pressure
        print(wrench_force_z)
        # Move the hand to front of the bottle
        whole_body.move_end_effector_pose(bottle_to_hand, _BOTTLE_TF)
        gripper.apply_force(_GRASP_FORCE)
        # Wait time for simulator's grasp hack. Not needed on actual robot
        rospy.sleep(2.0)
        # Move the hand up on end effector coordinate
        whole_body.move_end_effector_pose(hand_up, _HAND_TF)
        # Move the hand toward handover transfer position
        whole_body.move_end_effector_pose(L, _HAND_TF)
        # waiting for gripper torque change
        while wrench_force_z > -11:
            print(wrench_force_z)
            continue
        gripper.command(1.2)
        # Transit to initial posture
        omni_base.go_rel(-0.1, 0.0, 0.0, 10.0)
        whole_body.move_to_go()

    except:
        tts.say('Fail to grasp.')
        rospy.logerr('fail to grasp')
        sys.exit()
