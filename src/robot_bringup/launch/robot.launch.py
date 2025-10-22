#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    robot_battery = Node(
        package="robot_battery",
        executable="robot_battery",
    )

    robot_led = Node(
        package="robot_led",
        executable="robot_led",
    )

    robot_nav_server = Node(
        package="robot_nav",
        executable="robot_nav_server",
    )

    return LaunchDescription([
        robot_battery,
        robot_led,
        robot_nav_server
    ])