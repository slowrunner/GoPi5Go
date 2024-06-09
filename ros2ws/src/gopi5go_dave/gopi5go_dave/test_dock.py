#!/usr/bin/env python3

# FILE: test_dock.py


"""
	Undock, wait, Dock, wait
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from threading import Event,Thread

from dave_interfaces.srv import Dock, Undock
from dave_interfaces.msg import DockStatus

import sys
import traceback
import datetime as dt
import time

DEBUG = False
# uncomment following for debug 
DEBUG = True


DT_FORMAT = "%Y-%m-%d %H:%M:%S"


class TestDock(Node):

    def __init__(self):
        super().__init__('test_dock')

        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        printMsg = '** test_dock node started **'
        print(dtstr,printMsg)

        # Create a call back group to allow call backs while in a call back
        # self.dock_srv_done_event = Event()
        # self.cb_grp = ReentrantCallbackGroup()

        # subscriptions
        self.dock_status = DockStatus()
        self.sub_dock_status = self.create_subscription(
            DockStatus,
            'dock_status',
            self.dock_status_cb,
            qos_profile_sensor_data)
        #     callback_group=self.cb_grp)

        # action clients

        # service clients

        # self.undock_client = self.create_client(Undock, 'undock', callback_group=self.cb_grp)
        self.undock_svc_client = self.create_client(Undock, 'undock')
        self.undock_svc_future = None

        self.hz = 1   # execute test_dock_cb once every second
        # self.timer = self.create_timer(1.0/self.hz, self.test_main_cb, callback_group=self.cb_grp)
        self.timer = self.create_timer(1.0/self.hz, self.test_main_cb)
        self.delay = 0
        self.state = "init"


    def dock_status_cb(self,dock_status_msg):
        if DEBUG: #  and (self.dock_status.is_docked != dock_status_msg.is_docked):
            self.dock_status = dock_status_msg
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg = "dock_status_cb(): dock_status.is_docked is now {} ".format(self.dock_status.is_docked)
            print(dtstr,printMsg)
        if self.dock_status.is_docked:
            self.state = "docked"


    def send_undock_svc_req(self):
        dtstr = dt.datetime.now().strftime(DT_FORMAT)
        printMsg = "send_undock_svc_req()"
        print(dtstr, printMsg)
        self.undock_svc_req = Undock.Request()
        self.undock_svc_future = self.undock_svc_client.call_async(self.undock_svc_req)


    def test_main_cb(self):
        try:

            dtstr = dt.datetime.now().strftime(DT_FORMAT)
            printMsg = "dave_main_cb: state: {}  is_docked: {}".format(self.state, self.dock_status.is_docked)
            print(dtstr, printMsg)
            if self.state == "init":
                printMsg = "dave_main_cb: exec init"
                print(dtstr, printMsg)

            elif self.state == "ready_to_dock":
                printMsg = "dave_main_cb: exec ready_to_dock"
                print(dtstr, printMsg)
                self.delay += 1
                if self.delay > 60:
                    self.state = "docking"
                    self.delay = 0
            elif self.state == "docking":
                printMsg = "dave_main_cb: exec docking"
                print(dtstr, printMsg)
                    # if ...
            elif self.state == "docked":
                printMsg = "dave_main_cb: exec docked - delay: {}".format(self.delay)
                print(dtstr, printMsg)
                self.delay += 1
                if self.delay > 60:
                    self.delay = 0
                    if self.undock_svc_client.service_is_ready():
                        self.send_undock_svc_req()
                        self.state = "undocking"
            elif self.state == "undocking":
                printMsg = "dave_main_cb: exec undocking"
                print(dtstr, printMsg)
                if self.undock_svc_future.done == True:  # print("self.undock_svc_future:", self.undock_svc_future.__dict__)
                    print("undock_svc_future.result.success {}  is_docked: {}".format(self.undock_svc_future.result.success, self.undock_svc_future.result.is_docked))
                    if self.undock_svc_future.result.success == True:
                        self.state = "undocked"

            elif self.state == "undocked":
                printMsg = "dave_main_cb: exec undocked"
                print(dtstr, printMsg)
            else:
                printMsg = "dave_main_cb: exec else (self.state not a handled value)"
                print(dtstr, printMsg)


        except Exception as e:
            print("dave_main_cb: Exception:",str(e))
            traceback.print_exc(file=sys.stdout)

def main():
    rclpy.init(args=None)
    test_node = TestDock()
    # mtexecutor = rclpy.executors.MultiThreadedExecutor(8)
    # mtexecutor.add_node(test_node)
    # mtexecutor_thread = Thread(target=mtexecutor.spin, daemon=True)
    # mtexecutor_thread.start()
    try:
        rclpy.spin(test_node)
        # while rclpy.ok():
        #    time.sleep(10)
    except KeyboardInterrupt:
        print("test_node: main: KeyboardInterrupt")
        pass
    except ExternalShutdownException:
        print("test_node: main: ExternalShutdownException")
        pass
    finally:
        print("test_node: main: destroy_node")
        test_node.destroy_node()
        try:
            print("test_node: main: try_shutdown")
            rclpy.try_shutdown()
        except:
            print("test_node: main: try_shutdown exception")
            pass
    # mtexecutor_thread.join()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
