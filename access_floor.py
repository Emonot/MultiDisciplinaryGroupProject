#!/usr/bin/env python

import rospy
import rosservice
import rosparam

import tf
import time
from std_srvs.srv import Empty
from rosplan_knowledge_msgs.msg import *
from rosplan_knowledge_msgs.srv import *

from diagnostic_msgs.msg import KeyValue
from rosplan_interface_mapping.srv import *
from rosplan_dispatch_msgs.srv import *
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist

from navigation_2d_spot.srv import CreatePath, CreatePathResponse

query = []


# ROSPlan documentation 
"""
When working with Knowledge base, create this
d2 = KnowledgeUpdateServiceArrayRequest()
d2.update_type = [0,]

uint8 ADD_KNOWLEDGE=0
uint8 ADD_GOAL=1
uint8 REMOVE_KNOWLEDGE=2
uint8 REMOVE_GOAL=3
uint8 ADD_METRIC=4
uint8 REMOVE_METRIC=5

http://kcl-planning.github.io/ROSPlan//tutorials/tutorial_08

k.knowledge_type = 1

uint8 INSTANCE = 0
uint8 FACT = 1
uint8 FUNCTION = 2
uint8 EXPRESSION = 3
uint8 INEQUALITY = 4

https://kcl-planning.github.io/ROSPlan//documentation/knowledge/03_KnowledgeItem.html


"""

def verify_goal_reached(msg):

    print("wait before climbing stairs")
 
    goal_id = "1"
    #goal_status=int(msg.status_list[0])
    if (msg.status_list):

        goal_status = msg.status_list[0].status

        goal_id_array = msg.status_list[0].goal_id.id #ou 10
        goal_id_string = goal_id_array.split('-')
        goal_id = goal_id_string[1]
        print(goal_id)
 
    else:
        goal_status = 1
        goal_id = "0"

    if(goal_status == 3 and goal_id == "2"):

        print("climb stairs now")
        """

        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(10)
        move = Twist()

        move.linear.x = 0.5
        move.linear.y = 0
        move.linear.z = 0

        move.angular.x = 0
        move.angular.y = 0
        move.angular.z = 0
        pub.publish(move)
        """
        rospy.wait_for_service('climb_stairs')
        try:
            climb_stairs = rospy.ServiceProxy('climb_stairs')
            resp = climb_stairs()
            return resp.success
	    except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        rosparam.delete_param("/home/evrard/catkin_ws/src/spot_config/maps/robotarium_rough_5.yalm")
        new_map = rosparam.load_file("/home/evrard/catkin_ws/src/spot_config/maps/robotarium_floor.yaml")   
        rosparam.setParam(new_map)

        rosparam.delete_param("/home/evrard/catkin_ws/src/spot_config/config/waypoint.yalm")
        new_waipoints = rosparam.load_file("/home/evrard/catkin_ws/src/spot_config/config/waypoint_floor.yalm")
        rosparam.setParam(new_waipoints)

        

def node_init():
    rospy.init_node('waiting_before_climbing_stairs') # , anonymous=True

    rospy.Subscriber("/move_base/status",GoalStatusArray,verify_goal_reached)
    rospy.spin()

if __name__=="__main__":
    node_init()