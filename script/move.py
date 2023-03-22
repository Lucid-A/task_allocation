#! /usr/bin/python3

from math import *

import rospy as ros
import actionlib as act

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

'''
uint8 PENDING=0
uint8 ACTIVE=1
uint8 PREEMPTED=2
uint8 SUCCEEDED=3
uint8 ABORTED=4
uint8 REJECTED=5
uint8 PREEMPTING=6
uint8 RECALLING=7
uint8 RECALLED=8
uint8 LOST=9
'''

'''    
0,0: -0.75 0    z 
528: 2.76 -0.14 z
873: 0.85 -0.15 w
'''

if "__main__" == __name__:
    ros.init_node("my_send_goal_node", anonymous=True)
    cli = act.SimpleActionClient("move_base", MoveBaseAction)
    cli.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "tag"
    goal.target_pose.header.stamp = ros.Time.now()
    
    goal.target_pose.pose.position.x = 2.76
    goal.target_pose.pose.position.y = -0.14
    goal.target_pose.pose.position.z = 0

    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = 1
    goal.target_pose.pose.orientation.w = 0

    for i in range(3):
        cli.send_goal(goal)
        print("Current Goal")
        print(goal)

        wait = cli.wait_for_result(ros.Duration(60))
        result = cli.get_result()
        state = cli.get_state()

        print("Result Position")
        print(result)

        print(f"State: {state}")
    
    if not wait:
        ros.logerr("Didn't reach goal in 60s")
    elif GoalStatus.SUCCEEDED != state:
        ros.logerr("Failed!")
    else:
        ros.loginfo("Success")
