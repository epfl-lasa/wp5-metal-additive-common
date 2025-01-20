#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
# This script is used to publish waypoints manually on the /damage_string topic.
# The waypoints are read from a file and published one by one as damage areas.

# Author: Louis Munier
# Last update: 2024-10-20

import rospy
import yaml
import itertools
from std_msgs.msg import String

def main():
    """ Main function. """
    pub = rospy.Publisher('/damage_string', String, queue_size=10)
    rospy.init_node('waypoint_publisher', anonymous=True)
    rospy.loginfo('Waypoint publisher started.')

    # Read waypoints from a file
    yaml_file = rospy.get_param('~yaml_file', 'waypoints.yaml')
    waypoints = read_waypoints(yaml_file)

    # Publish waypoints
    publish_waypoints(pub, waypoints)

def read_waypoints(filename: str) -> dict:
    """ Read waypoints from a file. """
    with open(filename, 'r') as file:
        waypoints = yaml.safe_load(file)

    return waypoints

def pack_waypoint(waypoint: dict, id_damage: int) -> str:
    """ Create a waypoint message from a waypoint. """
    msg = String()
    msg.data = f"{id_damage}{waypoint['type']},"

    positions = [
        waypoint['positions']['start'],
        waypoint['positions']['stop']
    ]

    for coord in itertools.chain(*positions):
        msg.data += f"{coord},"

    return msg.data[:-1]  # Remove last comma

def publish_waypoints(publisher: rospy.Publisher, waypoints: dict):
    """ Publish waypoints on the /damage_string topic. """
    SEND_REPETITIONS = 10
    rate = rospy.Rate(50)  # 50 Hz

    # Publish waypoint
    for i, waypoint in enumerate(waypoints):
        waypoint_msg = pack_waypoint(waypoint, i)
        rospy.loginfo(f"Publishing waypoint: {waypoint_msg}")

        for i in range(SEND_REPETITIONS):
            publisher.publish(waypoint_msg)
            rate.sleep()

if __name__ == '__main__':
    main()
