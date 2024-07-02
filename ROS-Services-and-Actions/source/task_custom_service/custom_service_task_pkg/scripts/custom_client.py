#!/usr/bin/env python3

import sys
import rospy
from custom_service_task_pkg.srv import AddTwoNumber

def add_two_number_client(x, y):
    rospy.wait_for_service('add_two_numbers_service')
    try:
        add_two_number = rospy.ServiceProxy('add_two_numbers_service', AddTwoNumber)
        res = add_two_number(x, y)
        return res.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_number_client(x, y)))