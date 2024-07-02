#!/usr/bin/env python3


from custom_service_task_pkg.srv import AddTwoNumber, AddTwoNumberResponse
import rospy

def handle_add_two_numbers(req):
    print("Returning [%s + %s = %s]"%(req.num_1, req.num_2, (req.num_1 + req.num_2)))
    return AddTwoNumberResponse(req.num_1 + req.num_2)

def add_two_number_server():
    rospy.init_node('add_two_number_server')
    s = rospy.Service('add_two_numbers_service', AddTwoNumber, handle_add_two_numbers)
    print("Ready to add two numver.")
    rospy.spin()

if __name__ == "__main__":
    add_two_number_server()