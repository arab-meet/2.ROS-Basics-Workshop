#!/usr/bin/env python3

import sys
import rospy
from service_action_examples_package.srv import Square

def square_client(number):
    # Wait for the 'calculate_square' service to become available
    rospy.wait_for_service('calculate_square')
    try:
        # Create a handle to the service
        calculate_square = rospy.ServiceProxy('calculate_square', Square)
        # Call the service with the provided number
        response = calculate_square(number)
        # Return the result from the service response
        return response.square
    except rospy.ServiceException as e:
        # Print error if the service call fails
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    # Check if a number was provided as a command-line argument
    if len(sys.argv) == 2:
        number = int(sys.argv[1])
    else:
        print("Usage: square_client.py [number]")
        sys.exit(1)
    
    # Print the number being requested
    print("Requesting the square of %d" % number)
    # Call the service and print the result
    print("Square: %d" % square_client(number))
