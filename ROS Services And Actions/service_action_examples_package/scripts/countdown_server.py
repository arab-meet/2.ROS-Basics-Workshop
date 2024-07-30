#!/usr/bin/env python3

import rospy
import actionlib
import time
import service_action_examples_package.msg

class CountdownServer():
    # Create Feedback and Result messages
    def __init__(self):
        # Create the server
        self._action_server = actionlib.SimpleActionServer('countdown', service_action_examples_package.msg.CountdownAction, self.execute_callback, False)

        # Start the server
        self._action_server.start()
        rospy.loginfo("Starting Action Server")
    
        # Callback function to run after acknowledging a goal from the client
    def execute_callback(self, goal_handle):
        rospy.loginfo("Starting countdown…")

        # Initiate the feedback message's current_num as the action request's starting_num
        feedback_msg = service_action_examples_package.msg.CountdownFeedback()
        feedback_msg.current_num = goal_handle.starting_num

        while feedback_msg.current_num>0:
            # Publish feedback
            self._action_server.publish_feedback(feedback_msg)


            # Print log messages
            rospy.loginfo('Feedback: {0}'.format(feedback_msg.current_num))


            # Decrement the feedback message's current_num
            feedback_msg.current_num = feedback_msg.current_num - 1

            # Wait a second before counting down to the next number
            time.sleep(1)

        self._action_server.publish_feedback(feedback_msg)
        rospy.loginfo('Feedback: {0}'.format(feedback_msg.current_num))
        rospy.loginfo('Done!')

        result = service_action_examples_package.msg.CountdownResult()
        result.is_finished = True
        # Indicate that the goal was successful
        self._action_server.set_succeeded(result)

def main(args=None):
   # Init ROS1 and give the node a name
   rospy.init_node("countdown_server")
   countdown_server = CountdownServer()
   rospy.spin()

if __name__ == '__main__':
   main()
        
