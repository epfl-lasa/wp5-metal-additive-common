#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Service to control the state of the MoveIt scenario for the first experiments in Greece.

# Author: Louis Munier
# Last update: 2024-05-13

import sys
import rospy

from anima_rokae_service.srv import *


def mam_control_client(request):
    rospy.wait_for_service('/mam_control')

    try:
        # Create a handle to the service
        mam_control = rospy.ServiceProxy('/mam_control', MAMControl)

        # Call service
        if not mam_control(request):
            print(f"Service call failed without exception raised.")
            sys.exit()
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


def usage():
    msg_help = "Should be called with an int as a request :\n"
    msg_help += "0 - Reset to current state, step 1\n"
    msg_help += "1 - Go to home position\n"
    msg_help += "2 - Go to work position\n"
    msg_help += "3 - Start chosen scenario\n"
    msg_help += "4 - Stop chosen scenario"

    print(msg_help)


if __name__ == "__main__":
    request = None

    try:
        request = int(sys.argv[1])
    except:
        usage()

    if request is not None:
        mam_control_client(request)
