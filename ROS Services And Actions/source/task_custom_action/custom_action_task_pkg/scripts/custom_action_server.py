#!/usr/bin/env python3

import rospy
import actionlib
from  custom_action_task_pkg.msg import motor_actionAction, motor_actionGoal, motor_actionResult, motor_actionFeedback

class MotorPositionServer:
    
    def __init__(self):
        # Create the server
        self.motor_action_server = actionlib.SimpleActionServer('motor_position_action', motor_actionAction, self.execute_motor, False)

        # Start the server
        self.motor_action_server.start()
        rospy.loginfo("Server: starting motor move ")
    
        # Callback function to run after acknowledging a goal from the client
    def execute_motor(self, goal):
        feedback = motor_actionFeedback()
        result = motor_actionResult()

        # Initialize variables
        ticks = 0
        rate = rospy.Rate(5)  # 5 Hz
        success = False

        # Main loop to increment ticks and publish feedback
        while not rospy.is_shutdown():
            ticks += 1
            feedback.current_encoder_count = ticks
            
            self.motor_action_server.publish_feedback(feedback)
            rospy.loginfo("feadback now is:  " + str(feedback.current_encoder_count))
            if ticks >= goal.goal_encoder_count:
                success = True
                break

            rate.sleep()

        if success:
            result.final_encoder_count = ticks
            rospy.loginfo("Server: motor move completed")
            self.motor_action_server.set_succeeded(result)
        else:
            self.motor_action_server.set_aborted()


if __name__ == '__main__':
    rospy.init_node('motor_position_action_server_node')
    server = MotorPositionServer()
    rospy.spin()