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
from dataclasses import dataclass
from std_msgs.msg import Int16, Bool

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
    MAX = 80  # [mm/s]


class IntAOutput(Enum):
    MIN = 0  # min int16 unsigned
    MAX = 32767  # max int16 unsigned


@dataclass
class Step():
    class ID(Enum):
        STOP = 0
        GAS = 1
        LASER = 2
        WIRE_FEEDER = 3
        START = 4


class AnimaLaser:
    def __init__(self, str_task: str):
        """
        Main function that initializes the ROS node, MoveIt, and executes the trajectory.
        """
        self.__gas_state = False  # Enabled / Disabled
        self.__power = 0  # Power in % of the byte value
        self.__laser_state = False  # Enabled / Disabled
        self.__wire_feed_speed = 0  # Wire feeder speed in % of the byte value

        self.__map_functions = {
            'gas': self.__set_gas_state,
            'laser': self.__set_power_state,
            'wire_feeder': self.__set_wire_feeder_state
        }

        rospy.init_node('anima_laser_node')
        config_file_path = rospy.get_param('~config_file')
        self.config = self.__read_config_file(config_file_path)[str_task]

        self.__init_ros_msgs()

        self.current_step = Step.ID.STOP.value
        self.start_process = self.config["start"]
        if "stop" in self.config:
            self.stop_process = self.config["stop"]
        else:
            self.stop_process = dict(
                reversed(list(self.start_process.items()))
            )

        self.__set_power(self.config["power"])
        self.__set_wire_feed_speed(self.config["wire_feed_speed"])

    def start(self):
        for k, v in self.start_process.items():
            self.__map_functions[k](True)
            rospy.sleep(v)

        self.current_step = Step.ID.START.value

    def stop(self):
        if self.stop_process:
            for k, v in self.stop_process.items():
                self.__map_functions[k](False)
                rospy.sleep(v)
        else:
            for k, v in reversed(self.start_process.items()):
                self.__map_functions[k](False)
                rospy.sleep(v)

        self.current_step = Step.ID.STOP.value

    def __check_param(self, value, const):
        new_value = value

        if new_value > const.MAX.value:
            rospy.logwarn(const.NAME.value + " too high, clamp it to MAX.")
            new_value = const.MAX.value
        elif new_value < const.MIN.value:
            rospy.logwarn(const.NAME.value + " too low, clamp it to MIN.")
            new_value = const.MIN.value

        return new_value

    def __init_ros_msgs(self):
        rospack = rospkg.RosPack()
        file_path = rospack.get_path(
            'ros_modbus_device_driver'
        ) + '/../config/ANiMA.json'

        with open(file_path, 'r') as f:
            data = json.load(f)

        # Create ROS topics
        ns_topic = data["name"]
        self.pub_wire_feed_speed = rospy.Publisher(
            ns_topic + "/" + self.config["wire_feed_speed_topic"] + "/write",
            Int16,
            queue_size=1
        )

        self.pub_laser_power = rospy.Publisher(
            ns_topic + "/" + self.config["power_topic"] + "/write",
            Int16,
            queue_size=1
        )

        self.pub_laser_state = rospy.Publisher(
            ns_topic + "/" + self.config["laser_out_state_topic"] + "/write",
            Bool,
            queue_size=1
        )

        self.pub_gas_state = rospy.Publisher(
            ns_topic + "/" + self.config["gas_state_topic"] + "/write",
            Bool,
            queue_size=1
        )

        self.sub_laser_state = rospy.Subscriber(
            ns_topic + "/" + self.config["laser_in_state_topic"],
            Bool,
            self.__cbk_laser_state
        )

        rospy.sleep(1)
        self.pub_laser_power.publish(self.__power)
        self.pub_laser_state.publish(self.__laser_state)
        self.pub_wire_feed_speed.publish(self.__wire_feed_speed)
        self.pub_gas_state.publish(self.__gas_state)

    def __set_power(self, power):
        power_clamped = self.__check_param(power, LaserPower)
        self.__power = int(self.__interpolate_value(
            power_clamped, LaserPower
        ))

        self.__update()

    def __set_wire_feed_speed(self, wire_feed_speed):
        wire_feed_speed_clamped = self.__check_param(
            wire_feed_speed, WireFeedSpeed
        )
        self.__wire_feed_speed = int(self.__interpolate_value(
            wire_feed_speed_clamped, WireFeedSpeed
        ))

        self.__update()

    def __update(self):
        if self.current_step >= Step.ID.GAS.value:
            self.pub_gas_state.publish(self.__gas_state)

        # Manage laser state
        if not self.__laser_state:
            self.__power = 0

        self.pub_laser_state.publish(self.__laser_state)
        self.pub_laser_power.publish(self.__power)

        # Manage wire feeder state
        if self.current_step >= Step.ID.WIRE_FEEDER.value:
            self.pub_wire_feed_speed.publish(self.__wire_feed_speed)
        else:
            self.pub_wire_feed_speed.publish(0)

    def __interpolate_value(self, value, const):
        in_min, in_max = const.MIN.value, const.MAX.value
        out_min, out_max = IntAOutput.MIN.value, IntAOutput.MAX.value

        in_diff = in_max - in_min
        out_diff = out_max - out_min

        return out_min + out_diff * (value - in_min) / in_diff

    def __read_config_file(self, str_filepath: str):
        with open(str_filepath, 'r') as file:
            data = yaml.safe_load(file)

        return data

    def __set_gas_state(self, enable):
        rospy.loginfo("__set_gas_state")

        self.current_step = Step.ID.GAS.value

        self.__gas_state = enable
        self.__update()

        if not enable:
            self.current_step -= 1

    def __set_power_state(self, enable):
        rospy.loginfo("__set_power_state")
        self.current_step = Step.ID.LASER.value

        self.__laser_state = enable

        if enable:
            self.__set_power(self.config["power"])
        else:
            self.__set_power(0)

        self.__update()

        if not enable:
            self.current_step -= 1

    def __set_wire_feeder_state(self, enable):
        rospy.loginfo("__set_wire_feeder_state")
        self.current_step = Step.ID.WIRE_FEEDER.value

        if enable:
            self.__set_wire_feed_speed(self.config["wire_feed_speed"])
        else:
            self.__set_wire_feed_speed(0)

        self.__update()

        if not enable:
            self.current_step -= 1

    def __cbk_laser_state(self, msg):
        # rospy.loginfo(f"Laser state : {msg.data}")
        pass


if __name__ == "__main__":
    laser = AnimaLaser("welding")
    rospy.sleep(3)
    laser.start()
    laser.stop()
