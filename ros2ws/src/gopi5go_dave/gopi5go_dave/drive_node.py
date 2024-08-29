#!/usr/bin/env python3

# FILE: drive_node.py

"""
    Offers /drive_distance service

    dave_interfaces.srv.DriveDistance.srv
        # Request
        # distance to drive (meters), positive or negative
        float32 distance
        # positive speed m/s
        float32 speed
        ---
        # Result
        # status: goal reached: 0, stall occurred: 1, time expired: 2
        int8 status

    CLI:   ros2 service call /drive_distance dave_interfaces/srv/DriveDistance "{distance: 0.017, speed: 0.05}"

    Design:  Uses multi-threaded execution and a ReentrantCallback group 
             to allow the drive main callback, the drive service callback, and the motor status callback to be executing simultaneously:

             When /drive_distance service request arrives drive_distance_cb 
               - copies request msg local
               - sets drive_state to drive_distance_init
             drive_main_cb (coded state machine)
               - States: init, ready, drive_distance_init, drive_distance_active
                 - drive_distance_init:
                   - records current motor encoder positions (previously set in motor_status_cb)
                   - records current time
                   - publishes /cmd_vel twist to start motion
                 - drive_distance_active:
                   - watches for stop conditions
                     - requested distance is reached (encoder positions updated by motor_status_cb in separate thread)
                     - [motor has stalled] 
                     - (distance/speed + tolerance) time has passed)
                  - publish all stop /cmd_vel
                  - return drive_distance done and result [goal reached: 0, stall occurred: 1, time expired: 2]

             When /motor_status arrives motor_status_cb executes:
               - saves a copy of current encoder postions
               - saves motor stall status

"""


from dave_interfaces.srv import DriveDistance

from math import pi

from time import sleep
import datetime as dt

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import threading


DT_FORMAT = "%Y-%m-%d %H:%M:%S.%f"


class Drive(Node):

    def __init__(self):
        super().__init__('drive')
        sub_cb_grp = None
        svc_cb_grp = None
        main_cb_grp = None

        self.motor_status_sub = self.create_subscription(MotorStatusLR, 'motor/status', self.motor_status_cb, qos_profile=qos_profile_sensor_data)
        self.drive_distance_srv = self.create_service(DriveDistance, 'drive_distance', self.drive_distance_cb)
        self.hz = 10  # check for requests
        self.timer = self.create_timer( 1.0/self.hz, self.docking_main_cb, callback_group=main_cb_grp)
        # self.get_logger().info('docking_node.init()')
        dtstr = dt.datetime.now().strftime(DT_FORMAT)[:-3]
        printMsg = 'drive_node.init(): node initialized'
        print(dtstr,printMsg)


    def motor_status_cb(self,motor_status_msg):
        self.is_charging = battery_state_msg.charging
        if self.is_charging != self.prior_is_charging:  # something changed
            if (self.is_charging):        # was not charging, now charging
                self.is_docked = True
            else:                         # was charging, now not 
                self.is_docked = False
        self.prior_is_charging = self.is_charging

        # self.battery_volts = battery_state_msg.volts
        # self.get_logger().info('docking_node.battery_state_cb()')
        dtstr = dt.datetime.now().strftime(DT_FORMAT)[:-3]
        printMsg = 'docking_node.battery_state_cb(): is_charging: {}  is_docked: {}'.format(self.is_charging, self.is_docked)
        print(dtstr,printMsg)

    def drive_distance_cb(self, request, response):

        dtstr = dt.datetime.now().strftime(DT_FORMAT)[:-3]
        printMsg = 'drive_node.drive_distance_cb(): entry'
        print(dtstr,printMsg)
        # self.get_logger().info('docking_node.dock_cb() entry')
        wheel_dia_in_meters = self.egpg.WHEEL_CIRCUMFERENCE / 1000.0
        docking_speed_dps = int(DOCKING_SPEED_MPS * 360.0 / wheel_dia_in_meters )
        self.egpg.set_speed(docking_speed_dps)
        self.egpg.drive_cm(DOCKING_DIST_CM)


        response.is_docked = self.is_docked
        response.is_charging = self.is_charging
        response.success= (self.is_docked and self.is_charging)
        # self.get_logger().info('docking_node.dock_cb() return')
        dtstr = dt.datetime.now().strftime(DT_FORMAT)[:-3]
        printMsg = 'docking_node.dock_cb() exit: success: {} : is_charging: {}  is_docked: {}'.format(response.success, self.is_charging, self.is_docked)
        print(dtstr,printMsg)

        return response

    def docking_main_cb(self):
        # self.get_logger().info('docking_node.main_cb() entry')
        dtstr = dt.datetime.now().strftime(DT_FORMAT)[:-3]
        printMsg = 'docking_node.main_cb() entry: is_charging: {}  is_docked: {}'.format(self.is_charging, self.is_docked)
        print(dtstr,printMsg)

        try:
            dock_status = DockStatus()
            dock_status.is_docked = self.is_docked
            dock_status.is_charging = self.is_charging
            self.dock_status_pub.publish(dock_status)
            dtstr = dt.datetime.now().strftime(DT_FORMAT)[:-3]
            printMsg = 'docking_node.main_cb() publishing dock_status: is_charging: {}  is_docked: {}'.format(self.is_charging, self.is_docked)
            print(dtstr,printMsg)

        except Exception as e:
            print("docking_main_cb: ",str(e))
            sys.exit(1)

        # self.get_logger().info('docking_node.main_cb() done')


def main():
    rclpy.init()

    docking_node = Docking()
    executor = MultiThreadedExecutor()
    executor.add_node(docking_node)

    try:
        # rclpy.spin(docking_node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    docking_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




