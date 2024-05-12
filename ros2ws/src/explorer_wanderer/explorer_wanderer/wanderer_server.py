# written by Enrique Fernández-Laguilhoat Sánchez-Biezma and Daniel García López

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from explorer_interfaces.action import Wander

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from random import random


# ros2 action send_goal wander explorer_interfaces/action/Wander "{strategy: 1, map_completed_thres: 0.6}"

distance_from_wall = 0.8

class Subscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)
        self.subscription
        self.ranges = [0.0] * 180
        self.forward_distance = 1000.0
        self.left_forward_distance = 1000.0
        self.right_forward_distance = 1000.0
        self.left_distance = 1000.0
        self.back_distance = 1000.0
        self.right_distance = 1000.0

    def listener_callback(self, msg):
        # print("Forward distance: " + str(msg.ranges[0]))
        # print("Left distance: " + str(msg.ranges[90]))
        # print("Back distance: " + str(msg.ranges[180]))
        # print("Right distance: " + str(msg.ranges[270]))

        self.forward_distance = msg.ranges[0]
        self.left_distance = msg.ranges[90]
        self.back_distance = msg.ranges[180]
        self.right_distance = msg.ranges[270]
        self.left_forward_distance = msg.ranges[45]
        self.right_forward_distance = msg.ranges[315]


class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)


def reset_commands(command):
    """
    :param command: input Twist message
    :return: command: returns reset Twist message
    """
    command.linear.x = 0.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0.0

    return command


def check_ranges(subscriber):
    """
    returns boolean value of whether all sensors are above distance from wall.
    :param subscriber:
    :return: boolean value of whether the bot is clear to go forward
    :return: float value of the smallest sensor reading
    """
    rclpy.spin_once(subscriber)
    readings = [subscriber.forward_distance, subscriber.left_forward_distance, subscriber.right_forward_distance]
    min_value = min(readings)
    if subscriber.forward_distance < distance_from_wall:  # if there is a wall in front, dont go forward
        subscriber.get_logger().info("Obstacle detected")
        return False, min_value
    else:
        if (subscriber.right_forward_distance > distance_from_wall and
                subscriber.left_forward_distance > distance_from_wall):  # if theres no wall close to the sides,
            # go forward
            return True, min_value
        else:
            if (subscriber.left_forward_distance > subscriber.left_distance or  # if the robot is aiming away from
                    # the wall, go forward
                    subscriber.right_forward_distance > subscriber.right_distance):
                return True, min_value
            else:
                subscriber.get_logger().info("Obstacle detected")
                return False, min_value


def go_forward_until_obstacle(subscriber, publisher, command):
    """
    :param subscriber: subscriber object that is subscribed to /scan
    :param publisher: publisher object that that is publishing in /cmd_vel
    :param command: /Twist message object
    :return: nothing
    """
    command = reset_commands(command)

    while check_ranges(subscriber)[0]:  # while obstacles are not present or robot is aiming away from wall, go forward
        rclpy.spin_once(subscriber)
        speed = check_ranges(subscriber)[1] / 2.5  # robot speed depends on distance to closest obstacle
        if speed > 0.9:  # max speed
            speed = 0.9
        command.linear.x = speed
        publisher.get_logger().info('Going forward at %s m/s.' % round(speed, 2))
        publisher.publisher_.publish(command)

    command = reset_commands(command)
    publisher.publisher_.publish(command)


def rotate_until_clear(subscriber, publisher, command):
    """
    :param subscriber: subscriber object that is subscribed to /scan
    :param publisher: publisher object that that is publishing in /cmd_vel
    :param command: /Twist message object
    :return: nothing
    """

    command = reset_commands(command)
    rclpy.spin_once(subscriber)

    if subscriber.left_forward_distance < subscriber.right_forward_distance:  # if the robot has the wall to his left
        while subscriber.left_forward_distance < subscriber.left_distance or subscriber.forward_distance < distance_from_wall:
            rclpy.spin_once(subscriber)
            command.angular.z = -1.1 + (random()*0.3)
            publisher.publisher_.publish(command)
            publisher.get_logger().info("Rotating right...")
    else:
        while subscriber.right_forward_distance < subscriber.right_distance or subscriber.forward_distance < distance_from_wall:
            rclpy.spin_once(subscriber)
            command.angular.z = 1.1 - (random()*0.3)
            publisher.publisher_.publish(command)
            publisher.get_logger().info("Rotating left...")

    subscriber.get_logger().info("Clear.")

    command = reset_commands(command)
    publisher.publisher_.publish(command)


class WandererServer(Node):

    def __init__(self):
        super().__init__('wanderer_server')
        self._action_server = ActionServer(self,Wander,'wander',self.execute_callback)
        self.watchtower_subscription = self.create_subscription(Float32,'map_progress',self.watchtower_callback,10)
        self.watchtower_subscription  # prevent unused variable warning
        self.stop_wandering=False
        self.map_completed_thres=1.0 #Initialize threshold to max (100%)
        self.get_logger().info("Wanderer Server is ready")
        self.subscriber = Subscriber()
        self.publisher = Publisher()
        self.command = Twist()

    def watchtower_callback(self, msg):
        #If map_progress is higher than the threshold send stop wandering signal
        if msg.data>self.map_completed_thres:
            self.stop_wandering=True
        

    def execute_callback(self, goal_handle):
        self.get_logger().info("Wanderer Server received a goal")
        self.map_completed_thres=goal_handle.request.map_completed_thres
        self.get_logger().info("Map completed threshold set to: %s" %self.map_completed_thres)
        while not self.stop_wandering:  # main loop. The robot goes forward until obstacle, and then turns until its free to advance, repeatedly.
            go_forward_until_obstacle(self.subscriber, self.publisher, self.command)
            rotate_until_clear(self.subscriber, self.publisher, self.command)

        self.get_logger().info('Wandering Finished')
        goal_handle.succeed()
        return Wander.Result()


def main(args=None):
    rclpy.init(args=args)

    wanderer_server = WandererServer()

    rclpy.spin(wanderer_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
