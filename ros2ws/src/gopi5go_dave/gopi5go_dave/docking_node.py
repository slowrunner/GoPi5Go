#!/usr/bin/env python3

# FILE: docking_node.py

"""
    Offers two services /dock and /undock
    Publishes /dock_status topic


    dave_interfaces.srv.Dock.srv
        # Request
        ---
        # Result
        bool is_docked
        bool is_charging
        bool success

    dave_interfaces.srv.Undock.srv
        # Request
        ---
        # Result
        bool is_docked
        bool success

    dave_interfaces.msg.DockStatus.msg
        # Request
        ---
        # Result
        bool is_docked
        bool is_charging


    CLI:   ros2 service call /dock dave_interfaces/srv/Dock
           ros2 action call /undock dave_interfaces/srv/Undock
           ros2 topic echo /dock_status dave_interfaces/msg/DockStatus
"""


from dave_interfaces.srv import Dock, Undock
from dave_interfaces.msg import BatteryState, DockStatus
import sys
sys.path.insert(1,'/home/pi/GoPi5Go/plib')
from noinit_easygopigo3 import EasyGoPiGo3
from math import pi

from time import sleep
import datetime as dt

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import threading


DOCKING_SPEED_MPS = 0.05  # Docking speed in m/s
DOCKING_DIST_CM = -17.4  # cm
UNDOCKING_DIST_CM = 17.0 # cm

DT_FORMAT = "%Y-%m-%d %H:%M:%S.%f"


class Docking(Node):

    def __init__(self):
        super().__init__('docking')
        sub_cb_grp = None
        svc_cb_grp = None
        main_cb_grp = None

        # instantiate an EasyGoPiGo3 object without initializing default speed
        self.egpg = EasyGoPiGo3(use_mutex=True, noinit=True)

        self.battery_sub = self.create_subscription(BatteryState, 'battery_state', self.battery_state_cb, callback_group=sub_cb_grp, qos_profile=qos_profile_sensor_data)
        self.dock_status_pub = self.create_publisher(DockStatus, 'dock_status', qos_profile_sensor_data )
        self.dock_srv = self.create_service(Dock, 'dock', self.dock_cb, callback_group=svc_cb_grp)
        self.undock_srv = self.create_service(Undock, 'undock', self.undock_cb, callback_group=svc_cb_grp)
        self.hz = 1  # once per second
        self.timer = self.create_timer( 1.0/self.hz, self.docking_main_cb, callback_group=main_cb_grp)
        self.is_docked = False
        self.is_charging = False
        self.prior_is_charging = self.is_charging
        # self.get_logger().info('docking_node.init()')
        dtstr = dt.datetime.now().strftime(DT_FORMAT)[:-3]
        printMsg = 'docking_node.init(): is_charging: {}  is_docked: {}'.format(self.is_charging, self.is_docked)
        print(dtstr,printMsg)


    def battery_state_cb(self,battery_state_msg):
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

    def dock_cb(self, request, response):

        dtstr = dt.datetime.now().strftime(DT_FORMAT)[:-3]
        printMsg = 'docking_node.dock_cb() entry: is_charging: {}  is_docked: {}'.format(self.is_charging, self.is_docked)
        print(dtstr,printMsg)
        # self.get_logger().info('docking_node.dock_cb() entry')
        wheel_dia_in_meters = self.egpg.WHEEL_CIRCUMFERENCE / 1000.0
        docking_speed_dps = int(DOCKING_SPEED_MPS * 360.0 / wheel_dia_in_meters )
        self.egpg.set_speed(docking_speed_dps)
        self.egpg.drive_cm(DOCKING_DIST_CM)

        self.is_docked = True

        response.is_docked = self.is_docked
        response.is_charging = self.is_charging
        response.success= (self.is_docked and self.is_charging)
        # self.get_logger().info('docking_node.dock_cb() return')
        dtstr = dt.datetime.now().strftime(DT_FORMAT)[:-3]
        printMsg = 'docking_node.dock_cb() exit: success: {} : is_charging: {}  is_docked: {}'.format(response.success, self.is_charging, self.is_docked)
        print(dtstr,printMsg)

        return response

    def undock_cb(self, request, response):
        dtstr = dt.datetime.now().strftime(DT_FORMAT)[:-3]
        printMsg = 'docking_node.undock_cb() entry: is_charging: {}  is_docked: {}'.format(self.is_charging, self.is_docked)
        print(dtstr,printMsg)
        # self.get_logger().info('docking_node.undock_cb() entry')

        wheel_dia_in_meters = self.egpg.WHEEL_CIRCUMFERENCE / 1000.0
        docking_speed_dps = int(DOCKING_SPEED_MPS * 360.0 / wheel_dia_in_meters )
        self.egpg.set_speed(docking_speed_dps)
        self.egpg.drive_cm(UNDOCKING_DIST_CM)
        self.is_docked = False

        response.is_docked = self.is_docked
        response.success = True
        # self.get_logger().info('docking_node.undock_cb() return')
        dtstr = dt.datetime.now().strftime(DT_FORMAT)[:-3]
        printMsg = 'docking_node.undock_cb() exit: success: {} : is_charging: {}  is_docked: {}'.format(response.success, self.is_charging, self.is_docked)
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




