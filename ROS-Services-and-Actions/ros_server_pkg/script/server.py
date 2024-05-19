#!/usr/bin/env python3

import rospy
from ros_server_pkg.srv import MyService

def handle_request(request):
    rospy.loginfo("Received request: %s", request.input)
    response = "Received: " + request.input
    return response

def server():
    rospy.init_node('my_server')
    rospy.Service('my_service', MyService, handle_request)
    rospy.loginfo("Server is ready to receive requests.")
    rospy.spin()

if __name__ == '__main__':
    server()

