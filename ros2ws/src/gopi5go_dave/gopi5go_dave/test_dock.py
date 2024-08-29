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


DT_FORMAT = "%Y-%m-%d %H:%M:%S.%f"

TEST_DOCKED_DELAY = 60 # seconds
TEST_UNDOCKED_DELAY = 15 # seconds

class TestDock(Node):

    def __init__(self):
        super().__init__('test_dock')

        if DEBUG:
            dtstr = dt.datetime.now().strftime(DT_FORMAT)[:-3]
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

        self.dock_svc_client = self.create_client(Dock, 'dock')
        self.dock_svc_future = None

        self.hz = 1   # execute test_dock_cb once every second
        # self.timer = self.create_timer(1.0/self.hz, self.test_main_cb, callback_group=self.cb_grp)
        self.timer = self.create_timer(1.0/self.hz, self.test_main_cb)
        self.delay = 0
        self.state = "init"


    def dock_status_cb(self,dock_status_msg):
        if DEBUG: #  and (self.dock_status.is_docked != dock_status_msg.is_docked):
            self.dock_status = dock_status_msg
            dtstr = dt.datetime.now().strftime(DT_FORMAT)[:-3]
            printMsg = "dock_status_cb(): dock_status.is_docked is now {} ".format(self.dock_status.is_docked)
            print(dtstr,printMsg)
        if self.dock_status.is_docked:
            self.state = "docked"


    def send_undock_svc_req(self):
        if DEBUG:
            dtstr = dt.datetime.now().strftime(DT_FORMAT)[:-3]
            printMsg = "send_undock_svc_req()"
            print(dtstr, printMsg)
        self.undock_svc_req = Undock.Request()
        self.undock_svc_future = self.undock_svc_client.call_async(self.undock_svc_req)

    def send_dock_svc_req(self):
        if DEBUG:
            dtstr = dt.datetime.now().strftime(DT_FORMAT)[:-3]
            printMsg = "send_dock_svc_req()"
            print(dtstr, printMsg)
        self.dock_svc_req = Dock.Request()
        self.dock_svc_future = self.dock_svc_client.call_async(self.dock_svc_req)


    def test_main_cb(self):
        try:
            if DEBUG:
                dtstr = dt.datetime.now().strftime(DT_FORMAT)[:-3]
                printMsg = "dave_main_cb: state: {}  is_docked: {}".format(self.state, self.dock_status.is_docked)
                print(dtstr, printMsg)
            if self.state == "init":
                if DEBUG:
                    printMsg = "dave_main_cb: exec init"
                    print(dtstr, printMsg)

            elif self.state == "ready_to_dock":
                if DEBUG:
                    printMsg = "dave_main_cb: exec ready_to_dock - delay: {}".format(self.delay)
                    print(dtstr, printMsg)
                self.delay += 1
                if self.delay > TEST_UNDOCKED_DELAY:
                    self.state = "docking"
                    self.delay = 0
                    if self.dock_svc_client.service_is_ready():
                        self.send_dock_svc_req()
                        self.state = "docking"

            elif self.state == "docking":
                if DEBUG:
                    printMsg = "dave_main_cb: exec docking"
                    print(dtstr, printMsg)
                if self.dock_svc_future.done == True:  # print("self.undock_svc_future:", self.undock_svc_future.__dict__)
                    if DEBUG:
                        print("dock_svc_future.result.success {}  is_docked: {}".format(self.dock_svc_future.result.success, self.dock_svc_future.result.is_docked))
                    if self.dock_svc_future.result.success == True:
                        self.state = "docked"

            elif self.state == "docked":
                if DEBUG:
                    printMsg = "dave_main_cb: exec docked - delay: {}".format(self.delay)
                    print(dtstr, printMsg)
                self.delay += 1
                if self.delay > TEST_DOCKED_DELAY:
                    self.delay = 0
                    if self.undock_svc_client.service_is_ready():
                        self.send_undock_svc_req()
                        self.state = "undocking"

            elif self.state == "undocking":
                if DEBUG:
                    printMsg = "dave_main_cb: exec undocking"
                    print(dtstr, printMsg)
                    print("self.undock_svc_future:", self.undock_svc_future.__dict__)
                if self.undock_svc_future._done == True:  # print("self.undock_svc_future:", self.undock_svc_future.__dict__)
                    if DEBUG:
                        print("undock_svc_future.result._success {}  is_docked: {}".format(self.undock_svc_future._result.success, self.undock_svc_future._result.is_docked))
                    if self.undock_svc_future._result.success == True:
                        self.state = "ready_to_dock"
                else:
                    if self.dock_status.is_docked == False:
                        self.state = "ready_to_dock"
                        if DEBUG:
                            printMsg = "dave_main_cb: undock_svc_future not done, but self.dock_status.is_docked is False - forcing 'ready_to_dock' state"
                            print(dtstr, printMsg)

            elif self.state == "undocked":
                if DEBUG:
                    printMsg = "dave_main_cb: exec undocked"
                    print(dtstr, printMsg)

            else:    # PROBLEM IF HERE
                printMsg = "dave_main_cb: exec else (self.state not a handled value)"
                print(dtstr, printMsg)


        except Exception as e:
            print("dave_main_cb: Exception:",str(e))
            traceback.print_exc(file=sys.stdout)

def main():
    rclpy.init(args=None)
    test_node = TestDock()
    mtexecutor = rclpy.executors.MultiThreadedExecutor(8)
    mtexecutor.add_node(test_node)
    mtexecutor_thread = Thread(target=mtexecutor.spin, daemon=True)
    mtexecutor_thread.start()
    try:
        # rclpy.spin(test_node)
        while rclpy.ok():
           time.sleep(10)
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
