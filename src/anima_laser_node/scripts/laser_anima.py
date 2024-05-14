#!/usr/bin/env python
# -*- coding: utf-8 -*-
# This script is designed to be used with Moveit and share to ANIMA to
# test simple trajectories on their Rokae CR7 robot.

# Author: Louis Munier
# Last update: 2024-05-13

import rospkg
import rospy
import yaml
import json

from enum import Enum

# Parameters to be set
# ======================== ANIMA : Please set the following parameters ========================

# ======================== ANIMA : END of parameter region ========================


class LaserPower(Enum):
    NAME = "Laser Power"
    MIN = 0  # [Watt]
    MAX = 2000  # [Watt]


class WireFeedSpeed(Enum):
    NAME = "Wire Feeder Speed"
    MIN = 0  # [mm/s]
    MAX = 100  # [mm/s]


class IntAOutput(Enum):
    MIN = 0  # min int16 unsigned
    MAX = 32767  # max int16 unsigned


class AnimaLaser:
    def __init__(self, str_task: str):
        """
        Main function that initializes the ROS node, MoveIt, and executes the trajectory.
        """
        rospy.init_node('anima_laser_node')
        config_file_path = rospy.get_param('~config_file')
        config = self.read_config_file(config_file_path)

        self.set_power(config[str_task]["power"])
        self.set_wire_feed_speed(config[str_task]["wire_feed_speed"])

        self.init_ros_msgs()

    def check_param(self, value, const):
        new_value = value

        if new_value > const.MAX.value:
            rospy.logwarn(const.NAME.value + " too high, clamp it to MAX.")
            new_value = const.MAX.value
        elif new_value < const.MIN.value:
            rospy.logwarn(const.NAME.value + " too low, clamp it to MIN.")
            new_value = const.MIN.value

        return new_value

    def init_ros_msgs(self):
        rospack = rospkg.RosPack()
        file_path = rospack.get_path(
            'ros_modbus_device_driver'
        ) + '/../config/ANiMA.json'

        with open(file_path, 'r') as f:
            data = json.load(f)

        self.ros_modbus_namespace = data["name"]
        print(self.ros_modbus_namespace)

    def start(self):
        pass

    def stop(self):
        pass

    def set_power(self, power):
        power_clamped = self.check_param(power, LaserPower)
        self.__power = int(self.interpolate_value(
            power_clamped, LaserPower
        ))
        self.update()

    def set_wire_feed_speed(self, wire_feed_speed):
        wire_feed_speed_clamped = self.check_param(
            wire_feed_speed, WireFeedSpeed
        )
        self.__wire_feed_speed = int(self.interpolate_value(
            wire_feed_speed_clamped, WireFeedSpeed
        ))
        self.update()

    def update(self):
        pass

    def interpolate_value(self, value, const):
        in_min, in_max = const.MIN.value, const.MAX.value
        out_min, out_max = IntAOutput.MIN.value, IntAOutput.MAX.value

        in_diff = in_max - in_min
        out_diff = out_max - out_min

        return out_min + out_diff * (value - in_min) / in_diff

    def read_config_file(self, str_filepath: str):
        with open(str_filepath, 'r') as file:
            data = yaml.safe_load(file)

        return data


if __name__ == "__main__":
    laser = AnimaLaser("welding")
