#!/usr/bin/env python3

import rospy
from service_action_examples_package.srv import Square, SquareResponse

def handle_square(req):
    # Print the received request number
    rospy.loginfo("Received request: %ld", req.number)
    # Compute the square of the number
    result = req.number * req.number
    # Print the result of the computation
    rospy.loginfo("Returning response: %ld", result)
    # Return the result as a SquareResponse message
    return SquareResponse(result)

def square_server():
    # Initialize the ROS node with the name 'square_server'
    rospy.init_node('square_server')
    # Create a service named 'calculate_square' of type Square
    # This service will use the handle_square function to process requests
    s = rospy.Service('calculate_square', Square, handle_square)
    # Print that the service is ready to handle requests
    rospy.loginfo("Ready to calculate the square of a number.")
    # Keep the node running and processing requests
    rospy.spin()

if __name__ == "__main__":
    # Start the square server
    square_server()
