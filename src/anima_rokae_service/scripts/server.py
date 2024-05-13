#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Service to control the state of the MoveIt scenario for the first experiments in Greece.

# Author: Louis Munier
# Last update: 2024-05-13

import rospy
import sys

from moveit_anima import AnimaMoveit
from enum import Enum
from anima_rokae_service.srv import *

NAME = 'mam_control_server'


class State(Enum):
    RESET = 0
    HOMING = 1
    WORKING = 2
    START = 3
    STOP = 4


CURRENT_STATE = State.HOMING
ANIMA_MOVEIT = AnimaMoveit()


def mam_control(req):
    state = None

    if type(req) == int:
        state = req
    else:
        try:
            state = req.state
        except rospy.ServiceException as e:
            print(f"Not a valide input to mam_control(): {e}")
            sys.exit()

    if state == State.RESET.value:
        reset_task()
    elif state == State.HOMING.value:
        go_home()
    elif state == State.WORKING.value:
        go_work()
    elif state == State.START.value:
        start_task()
    elif state == State.STOP.value:
        stop_task()
    else:
        rospy.logerr("No such option.")
        return MAMControlResponse(False)

    return MAMControlResponse(True)


def reset_task():
    rospy.loginfo("reset_task")
    global CURRENT_STATE

    prev_state = CURRENT_STATE
    CURRENT_STATE = State.RESET

    # Begin by stoping current task
    stop_task()

    # Ask to check if we would like to go back to previous work
    if prev_state.name == State.START.name:
        prev_state == State.WORKING

    answer = input(
        f"Would you like to go to your previous state : {prev_state.name} ? [y/N]"
    )

    if answer.lower() == 'y':
        mam_control(prev_state.value)


def go_home():
    rospy.loginfo("go_home")
    global CURRENT_STATE
    global ANIMA_MOVEIT

    CURRENT_STATE = State.HOMING
    ANIMA_MOVEIT.move_group.set_joint_value_target(
        [0, 0, 0, 0, 0, 0]
    )
    ANIMA_MOVEIT.plan_display_move()


def go_work():
    rospy.loginfo("go_work")
    global CURRENT_STATE
    global ANIMA_MOVEIT

    CURRENT_STATE = State.WORKING
    init_scenario()


def start_task():
    rospy.loginfo("start_task")
    global CURRENT_STATE
    global ANIMA_MOVEIT

    CURRENT_STATE = State.START
    init_scenario()
    ANIMA_MOVEIT.plan_execute_trajectory()


def stop_task():
    rospy.loginfo("stop_task")
    global CURRENT_STATE
    global ANIMA_MOVEIT

    CURRENT_STATE = State.STOP
    ANIMA_MOVEIT.move_group.stop()


def init_scenario():
    rospy.loginfo("init_scenario")
    global ANIMA_MOVEIT

    if ANIMA_MOVEIT.ID_SCENARIO is None:
        msg_scenario = "Which scenario do you want to perform ? [1, 2]\n"
        msg_scenario += "1 - Scenario where lines are spaced but parallels\n"
        msg_scenario += "2 - Scenario where lines are stacked\n\n"

        answer = None
        while answer != '1' and answer != '2':
            answer = input(msg_scenario)

        if answer == '1' or answer == '2':
            ANIMA_MOVEIT.set_scenario(int(answer))
        else:
            rospy.logerr("No such scenario.")

    # Generate and move to initial pose
    ANIMA_MOVEIT.generate_init_pose()
    ANIMA_MOVEIT.init_pose = ANIMA_MOVEIT.apply_offset_to_pose(
        ANIMA_MOVEIT.init_pose, ANIMA_MOVEIT.EE_OFFSET, angle_rad=False
    )
    ANIMA_MOVEIT.move_group.set_pose_target(ANIMA_MOVEIT.init_pose)
    ANIMA_MOVEIT.plan_display_move()


def mam_control_server():
    rospy.Service('mam_control', MAMControl, mam_control)

    # spin() keeps Python from exiting until node is shutdown
    rospy.spin()


if __name__ == "__main__":
    mam_control_server()
