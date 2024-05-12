
# FILE: explorer_wanderer/wanderer.py
# PURPOSE: Wander using LIDAR /scan topic distances to avoid obstacles
# REF: https://github.com/DaniGarciaLopez/ros2_explorer/tree/main
# HISTORY:
#  - Originally written by Enrique Fernández-Laguilhoat Sánchez-Biezma
#  - Nov 2023: slowrunner: Modified:
#    - scan range[0] faces back on GoPiGo3 robot Dave
#    - to average /scan ranges to tolerate LIDAR returning ocaisional zero values
#    - replaced explicit qos profile of "10" with qos_profile_sensor_data, which is 10 with BEST_EFFORT
#    - Lowered allowed distance from obstacles to 350mm (GoPiGo3 safe turning circle is 140mm)
#    - Lowered max_speed to to prevent tip-over when stopping GoPiGo3 robot HumbleDave
#    - Change algorithm that lowers speed when closer than the minimum obstacle distance_to_wall
#    - original detection algorithm allowed bot to get too close to a side wall
#    - changed obstacle detection to include right and left side distance to allow wandering closer to wallss
#      (original with 0.8m distance_to_walls tended to stay in central area of playground
#    - New algorithm allows closer to wall wandering, BUT forward obstacles roughly the width of robot may not be cleared well
#    - New algorithm when boxed in can cause left/right/left/right oscillation when moving forward after rotation would work well
#    - Added TTS begin, forward, and rotating announcements

from random import random

import rclpy
from rclpy.node import Node

# Added slowrunner
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from statistics import mean
import subprocess
import time

# slowrunner: changed from 0.8 meters
# distance_from_wall = 0.8  # Original must be this high or side wall collisions happen when rotating
distance_from_wall = 0.350

# Average each range over NUM_READINGS of non-zero scans
NUM_READINGS = 3      # at 9Hz looking for 3 non-zero readings
NUM_RANGES = 561
MAX_SPEED = 0.15 # m/s
i_forward = int(NUM_RANGES/2)-1  # may not be exact front if even number of ranges
i_left = int(NUM_RANGES * 0.75)
i_left_forward = int((i_forward + i_left)/2)  # half way between forward and left
i_right = int(NUM_RANGES/4)
i_right_forward = int((i_forward + i_right)/2)  # half way between forward and right
i_back = NUM_RANGES - 1

# Start slowing down at two times distance from wall
# SLOWING_FACTOR = (2.0 * distance_from_wall) / MAX_SPEED  # (2*0.35)/0.1 = 7
SLOWING_FACTOR = (1.5 * distance_from_wall) / MAX_SPEED  # (1.5*0.35)/0.15 = 3.5

class Subscriber(Node):

    def __init__(self):
        super().__init__('subscriber')

        # slowrunner set explicit qos
        # self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, qos_profile_sensor_data)
        self.subscription

        # slowrunner changed before scan distances 
        self.forward_distance = 5.0
        self.left_forward_distance = 5.0
        self.right_forward_distance = 5.0
        self.left_distance = 5.0
        self.back_distance = 5.0
        self.right_distance = 5.0

        # Create arrays to hold non-zero scan values
        self.forward = []
        self.left_forward = []
        self.right_forward = []
        self.left = []
        self.back = []
        self.right = []

    def listener_callback(self, msg):
        # subscriber.get_logger().info("number of ranges in scan: " + str(len(msg.ranges)))
        # subscriber.get_logger().info("Forward distance: " + str(msg.ranges[0]))
        # subscriber.get_logger().info("Left distance: " + str(msg.ranges[90]))
        # subscriber.get_logger().info("Back distance: " + str(msg.ranges[180]))
        # subscriber.get_logger().info("Right distance: " + str(msg.ranges[270]))

        # ignore zero scan ranges from LIDAR
        if  msg.ranges[i_forward] > 0:
            self.forward += [msg.ranges[i_forward]]
            if len(self.forward) > NUM_READINGS: self.forward = self.forward[-NUM_READINGS:]

        if msg.ranges[i_left] > 0:
            self.left += [msg.ranges[i_left]]
            if len(self.left) > NUM_READINGS: self.left = self.left[-NUM_READINGS:]

        # self.back_distance = msg.ranges[180]
        if msg.ranges[i_back] > 0:
            self.back += [msg.ranges[i_back]]
            if len(self.back) > NUM_READINGS: self.back = self.back[-NUM_READINGS:]

        # self.right_distance = msg.ranges[270]
        if msg.ranges[i_right] > 0:
            self.right += [msg.ranges[i_right]]
            if len(self.right) > NUM_READINGS: self.right = self.right[-NUM_READINGS:]

        # self.left_forward_distance = msg.ranges[45]
        if msg.ranges[i_left_forward] > 0:
            self.left_forward += [msg.ranges[i_left_forward]]
            if len(self.left_forward) > NUM_READINGS: self.left_forward = self.left_forward[-NUM_READINGS:]

        # self.right_forward_distance = msg.ranges[315]
        if msg.ranges[i_right_forward] > 0:
            self.right_forward += [msg.ranges[i_right_forward]]
            if len(self.right_forward) > NUM_READINGS: self.right_forward = self.right_forward[-NUM_READINGS:]


        if len(self.forward) > 1: self.forward_distance = mean(self.forward)
        if len(self.left) > 1: self.left_distance = mean(self.left) 
        # self.back_distance = msg.ranges[180]
        if len(self.back) > 1: self.back_distance = mean(self.back) 
        # self.right_distance = msg.ranges[270]
        if len(self.right) > 1: self.right_distance = mean(self.right) 
        # self.left_forward_distance = msg.ranges[45]
        if len(self.left_forward) > 1: self.left_forward_distance = mean(self.left_forward) 
        # self.right_forward_distance = msg.ranges[315]
        if len(self.right_forward) > 1: self.right_forward_distance = mean(self.right_forward) 


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
    
    for i in range(3):  # collect average of three readings
        rclpy.spin_once(subscriber)

    # include left, left fwd, fwd, right frwd, right in readings to be considered
    readings = [subscriber.forward_distance, subscriber.left_forward_distance, subscriber.left_distance, subscriber.right_forward_distance, subscriber.right_distance]

    # Original: only consider left fwd, fwd, right fwd readings
    """
    rclpy.spin_once(subscriber)
    readings = [subscriber.forward_distance, subscriber.left_forward_distance, subscriber.right_forward_distance]
    """

    subscriber.get_logger().info("readings: left {:2.3f}  lft_fwd {:2.3f}  fwd {:2.3f}   rt_fwd {:2.3f}   right {:2.3f}  back {:2.3f}".format( \
                 subscriber.left_distance, subscriber.left_forward_distance, subscriber.forward_distance, subscriber.right_forward_distance, \
                 subscriber.right_distance, subscriber.back_distance))
    min_value = min(readings)

    # NEW Algorithm
    if (subscriber.forward_distance < distance_from_wall):  # if there is a wall in front, dont go forward
          # subscriber.get_logger().info("\n*** forward obstacle at {:.3f} meters".format(subscriber.forward_distance))
          phrase="Forward obstacle {:.3f} meters".format(subscriber.forward_distance)
          subscriber.get_logger().info(phrase)
          return False, min_value

    elif (subscriber.right_forward_distance > distance_from_wall and
          subscriber.left_forward_distance > distance_from_wall):  # no front wall 
          # If left or right distance will not allow turn, we could be in trouble and not know it
          if (subscriber.right_distance < distance_from_wall or
              subscriber.left_distance  < distance_from_wall):
              phrase="Close on Left {:.3f} or right {:.3f} meters".format( \
                  subscriber.left_distance, subscriber.right_distance)
              subscriber.get_logger().info(phrase)
              return False, min_value

          else:   # and no side walls
              phrase="No Obstacles"
              subscriber.get_logger().info(phrase)
              return True, min_value
    else:
        phrase="Forward left {:.3f} or right obstacle {:.3f} meters".format( \
                  subscriber.left_forward_distance, subscriber.right_forward_distance)
        subscriber.get_logger().info(phrase)
        return False, min_value

    # Original Algorithm
    """
    if subscriber.forward_distance < distance_from_wall:  # if there is a wall in front, dont go forward
        subscriber.get_logger().info("Forward Obstacle detected")
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
                subscriber.get_logger().info("Right or Left Forward Obstacle detected")
                return False, min_value
    """


def go_forward_until_obstacle(subscriber, publisher, command):
    """
    :param subscriber: subscriber object that is subscribed to /scan
    :param publisher: publisher object that that is publishing in /cmd_vel
    :param command: /Twist message object
    :return: nothing
    """
    command = reset_commands(command)

    # check_ranges() returns (obstacle: true/false, distance)
    while check_ranges(subscriber)[0]:  # while obstacles are not present go forward
        rclpy.spin_once(subscriber)
        # speed = check_ranges(subscriber)[1] / 5  # robot speed depends on distance to closest obstacle
        speed = check_ranges(subscriber)[1] / SLOWING_FACTOR  # robot speed depends on distance to closest obstacle
        if speed > MAX_SPEED:  # max speed
            speed = MAX_SPEED
        command.linear.x = speed
        publisher.get_logger().info('*** Command fwd at %s m/s.' % round(speed, 2))
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
    for i in range(3):  # collect average of three readings
        rclpy.spin_once(subscriber)

    publisher.get_logger().info("rotate: readings: left {:2.3f}  lft_fwd {:2.3f}  fwd {:2.3f}   rt_fwd {:2.3f}   right {:2.3f}  back {:2.3f}".format( \
                 subscriber.left_distance, subscriber.left_forward_distance, subscriber.forward_distance, subscriber.right_forward_distance, \
                 subscriber.right_distance, subscriber.back_distance))
    
    # NEW ALGORITHM
    # if the robot has away to right wall ahead
    if subscriber.left_forward_distance < subscriber.right_forward_distance and \
       min(subscriber.left_forward_distance, subscriber.forward_distance, subscriber.right_forward_distance, \
           subscriber.left_distance, subscriber.right_distance ) < distance_from_wall :  
        while min(subscriber.left_forward_distance, subscriber.forward_distance, subscriber.right_forward_distance, \
                  subscriber.left_distance, subscriber.right_distance)  < distance_from_wall:
            rclpy.spin_once(subscriber)
            if subscriber.right_distance < distance_from_wall:
                cw_ccw = 1   # turn left even though wall angles away 
            else:
                cw_ccw = -1
            command.angular.z = cw_ccw * (1.1 - (random()*0.3))
            publisher.publisher_.publish(command)
            if cw_ccw > 0:
                phrase = "Rotating left, wall on right..."
            else:
                phrase = "Rotating right..."
            publisher.get_logger().info(phrase)



    # if the robot has away to left wall ahead
    if subscriber.left_forward_distance > subscriber.right_forward_distance and \
       min(subscriber.left_forward_distance, subscriber.forward_distance, subscriber.right_forward_distance, \
           subscriber.left_distance, subscriber.right_distance ) < distance_from_wall :  
        while min(subscriber.left_forward_distance, subscriber.forward_distance, subscriber.right_forward_distance, \
                  subscriber.left_distance, subscriber.right_distance)  < distance_from_wall:
            rclpy.spin_once(subscriber)
            if subscriber.left_distance < distance_from_wall:
                cw_ccw = -1   # turn righteven though wall angles away 
            else:
                cw_ccw = 1
            command.angular.z = cw_ccw * (1.1 - (random()*0.3))
            publisher.publisher_.publish(command)
            if cw_ccw > 0:
                phrase = "Rotating left..."
            else:
                phrase = "Rotating right, wall on left..."
            publisher.get_logger().info(phrase)


    else:  # wall is either exactly in front, or is on right, left or right and left!!  Catch all - turn till something opens up.
        turn_cnt = 0
        while min(subscriber.left_forward_distance, subscriber.forward_distance, subscriber.right_forward_distance, \
                  subscriber.left_distance, subscriber.right_distance)  < distance_from_wall:
            rclpy.spin_once(subscriber)
            turn_cnt += 1
            if turn_cnt < 100:
                if subscriber.left_distance < distance_from_wall:
                    cw_ccw = -1  # turn right
                else:
                    cw_ccw = 1   # turn left
                command.angular.z = cw_ccw * (1.1 - (random()*0.3))
                publisher.publisher_.publish(command)
                if cw_ccw > 0:
                    phrase = "Rotating left..."
                else:
                    phrase = "Rotating right..."
                publisher.get_logger().info(phrase)


            else:
                command = reset_commands(command)
                publisher.publisher_.publish(command)
                phrase = "I'm trapped, need assistance..."
                publisher.get_logger().info(phrase)
                time.sleep(5)

    # Original Algorithm
    """
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
    """

    publisher.get_logger().info("\n*** Clear.")

    command = reset_commands(command)
    publisher.publisher_.publish(command)


def say(phrase,vol=100):

    phrase = phrase.replace("I'm","I m")
    phrase = phrase.replace("'","")
    phrase = phrase.replace('"',' quote ')
    phrase = phrase.replace('*',"")

    # subprocess.check_output(['espeak-ng -s150 -ven-us+f5 -a'+str(vol)+' "%s"' % phrase], stderr=subprocess.STDOUT, shell=True)
    subprocess.check_output(['espeak-ng -s300 -a'+str(vol)+' "%s"' % phrase], stderr=subprocess.STDOUT, shell=True)


def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()
    publisher = Publisher()
    command = Twist()

    say("I'm Humble Dave. Starting Ross 2 wanderer",vol=200)
    time.sleep(2)

    while 1:  # main loop. The robot goes forward until obstacle, and then turns until its free to advance, repeatedly.
        time.sleep(1)
        say("Going forward",vol=50)
        go_forward_until_obstacle(subscriber, publisher, command)
        say("Something too close", vol=50)
        rotate_until_clear(subscriber, publisher, command)

    # rclpy.spin(subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
