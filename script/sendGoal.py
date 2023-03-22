#! /usr/bin/python3

import rospy as ros
import actionlib as act
import message_filters as mf

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from task_allocation.msg import StringStamped
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from IR_Camera.srv import ir_srv
from ocr.srv import ocr_srv
from IR_Camera.msg import IR_Camera_data

class SendGoal():
    def __init__(self):
        self.pub_result = ros.Publisher("/task_result", String, queue_size=10)

        self.sub_task = mf.Subscriber("/my_task", StringStamped)
        self.sub_goal = mf.Subscriber("/my_goal", PoseStamped)
        
        self.ts = mf.TimeSynchronizer([self.sub_task, self.sub_goal], 10)
        self.ts.registerCallback(self.callback)
        
        self.cli = act.SimpleActionClient("move_base", MoveBaseAction)
        self.cli.wait_for_server()
        ros.loginfo("Get Move Base Server!")

        ros.wait_for_service("ir_service")
        ros.loginfo("Get IR_Camera Server!")
        
        ros.wait_for_service("ocr_service")
        ros.loginfo("Get OCR Server!")

        self.ir_srv = ros.ServiceProxy("ir_service", ir_srv)
        self.ocr_srv = ros.ServiceProxy("ocr_service", ocr_srv)

    def callback(self, task_msg, goal_msg):
        ros.loginfo(f"task_msg: {task_msg.data}, goal_msg: ({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f})")
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "tag"
        goal.target_pose.header.stamp = ros.Time.now()
        goal.target_pose.pose = goal_msg.pose
        
        print("Current Goal")
        print(goal)

        status = -1
        break_flag = 10
        while GoalStatus.SUCCEEDED != status and break_flag > 0:
            break_flag = break_flag - 1

            self.cli.send_goal(goal)

            wait = self.cli.wait_for_result(ros.Duration(10))
            status = self.cli.get_state()
            goal_result = self.cli.get_result()

            if not wait:
                ros.logerr("Didn't reach goal in 60s")
            elif status != GoalStatus.SUCCEEDED:
                ros.logwarn(f"Status is {status}")
            else:
                result = String()
                if "A" == task_msg.data:
                    msg = ros.wait_for_message("/usb_cam_head/image_raw", Image)
                    result.data = self.ocr_srv(msg).text
                elif "B" == task_msg.data:
                    msg = ros.wait_for_message("/IR_Camera/data", IR_camera_data)
                    result.data = self.ir_srv(msg).result
                else:
                    result.data = ""

                ros.loginfo(f"Task result: {result.data}")
                self.pub_result.publish(result)
                break
        
        result = String()
        result.data = ""
        self.pub_result.publish(result)
    
if "__main__" == __name__:
    ros.init_node("my_send_goal_node", anonymous=True)
    SendGoal()
    ros.spin()
