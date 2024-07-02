#!/usr/bin/env python

import rospy
import actionlib
from  custom_action_task_pkg.msg import motor_actionAction, motor_actionGoal, motor_actionResult, motor_actionFeedback

def feedback_motor_callback(feedback):
    rospy.loginfo("Current tick: %d" % feedback.current_encoder_count)

def motor_action_client():
    client = actionlib.SimpleActionClient('motor_position_action', motor_actionAction)
    client.wait_for_server()

    goal = motor_actionGoal()
    goal.goal_encoder_count = 5  

    client.send_goal(goal, feedback_cb=feedback_motor_callback)

    client.wait_for_result()

    result = client.get_result()
    rospy.loginfo("Final tick: %d" % result.final_encoder_count)

if __name__ == '__main__':
    rospy.init_node('my_action_client_node')
    motor_action_client()
