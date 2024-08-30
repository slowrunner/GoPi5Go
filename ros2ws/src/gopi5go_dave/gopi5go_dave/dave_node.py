#!/usr/bin/env python3

# FILE: dave_node.py


"""
	Undock when charging current is <175ma
        Dock when voltage is < 10v

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
from dave_interfaces.msg import BatteryState, DockStatus

import sys
sys.path.insert(1,'/home/pi/GoPi5Go/plib')
import daveDataJson

import traceback
import logging
import datetime as dt
import time

DEBUG = False
# uncomment following for debug 
# DEBUG = True


LIFELOGFILE = "/home/pi/GoPi5Go/logs/life.log"


UNDOCK_AT_MILLIAMPS =  -175
DOCK_AT_VOLTS       =   10
# Testing
# UNDOCK_AT_MILLIAMPS =  -700
# DOCK_AT_VOLTS       =   11.2

DT_FORMAT = "%Y-%m-%d %H:%M:%S"


class DaveNode(Node):

    def __init__(self):
        super().__init__('dave_node')

        self.lifeLog = logging.getLogger(__name__)
        self.lifeLog.setLevel(logging.INFO)

        self.loghandler = logging.FileHandler(LIFELOGFILE)
        self.logformatter = logging.Formatter('%(asctime)s|%(filename)s| %(message)s',"%Y-%m-%d %H:%M")
        self.loghandler.setFormatter(self.logformatter)
        self.lifeLog.addHandler(self.loghandler)

        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        printMsg = '---- GoPi5Go-Dave node started - Undock:{:d} mA  Dock:{:.2f} v'.format(int(UNDOCK_AT_MILLIAMPS), DOCK_AT_VOLTS )
        print(dtstr,printMsg)
        self.lifeLog.info(printMsg)

        self.battery_state_sub = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_state_cb,
            qos_profile_sensor_data)    # best effort depth 10 

        self.battery_state = BatteryState()
        self.docked_at_Vbatt = 0.0

        self.sub = self.create_subscription(
            DockStatus,
            'dock_status',
            self.dock_status_cb,
            qos_profile_sensor_data)    # best effort depth 10 

        self.dock_status = DockStatus()

        # action clients

        # service clients
        self.dock_svc_client = self.create_client(Dock, 'dock')
        self.dock_svc_request = None
        self.dock_svc_future = None

        self.undock_svc_client = self.create_client(Undock, 'undock')
        self.undock_svc_request = None
        self.undock_svc_future = None

        self.hz = 1   # execute dave_cb once every second
        self.timer = self.create_timer(1.0/self.hz, self.dave_main_cb)   # call dave_main_cb once every period

        self.state = "init"
        self.prior_state = self.state
        self.last_dock_time = dt.datetime.strptime(daveDataJson.getData("lastDockingTime"),DT_FORMAT)
        self.last_undock_time = dt.datetime.strptime(daveDataJson.getData("lastDismountTime"),DT_FORMAT)
        if DEBUG:
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            print(dtstr,"DaveNode.init: last_dock_time:   ",self.last_dock_time.strftime(DT_FORMAT))
            print(dtstr,"               last_undock_time: ",self.last_undock_time.strftime(DT_FORMAT))
        self.all_req_topic_cb_rx = [False, False]  # /battery_state, /dock_status

    def battery_state_cb(self,battery_state_msg):
        self.battery_state = battery_state_msg
        self.all_req_topic_cb_rx[0] = True  # /battery_state, /dock_status
        if DEBUG:
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg = "battery_state_cb(): battery_state.charging {} .watts {:.1f}".format(self.battery_state.charging, self.battery_state.watts)
            print(dtstr,printMsg)

    def dock_status_cb(self,dock_status_msg):
        self.dock_status = dock_status_msg
        self.all_req_topic_cb_rx[1] = True  # /battery_state, /dock_status
        if DEBUG:
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg = "dock_status_cb(): dock_status.is_docked {} ".format(self.dock_status.is_docked)
            print(dtstr,printMsg)

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



    def dave_main_cb(self):
        try:

            # evaluate dave.state validity
            if DEBUG:
                dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                printMsg = "dave_main_cb(): executing"
                print(dtstr, printMsg)
                printMsg = "dave_main_cb(): dave.state = {}".format(self.state)
                print(dtstr, printMsg)


            if (self.state == "init"):       # Handle startup - wait for subcriptions, then figure out if docked or undocked
                if (self.all_req_topic_cb_rx[0] == False) or (self.all_req_topic_cb_rx[1] == False):
                  if DEBUG:
                    dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    printMsg = "dave_main_cb(): waiting for all_req_topic_cb_rx"
                    print(dtstr, printMsg)
                    printMsg = "                battery_state, dock_status = {}".format(self.all_req_topic_cb_rx)
                    print(dtstr, printMsg)
                elif self.battery_state.charging:     # init and all subcribers have received topics
                    self.prior_state = self.state
                    self.state = "docked"
                else:                              # PROBABLY UNDOCKED - Cannot be certain from only charging status
                    self.prior_state = self.state
                    self.state = "ready_to_dock"

            elif (self.state == "docking"):
                if DEBUG:
                    printMsg = "dave_main_cb: exec docking"
                    print(dtstr, printMsg)
                    print("self.dock_svc_future:", self.dock_svc_future.__dict__)
                    print("dock_status.is_docked {}  battery_state.charging {}".format(self.dock_status.is_docked, self.battery_state.charging))
                if self.dock_svc_future.done() == True:  # print("self.undock_svc_future:", self.undock_svc_future.__dict__)
                    # if DEBUG:
                    #    print("dock_svc_future.result.success {}  is_docked: {}".format(self.dock_svc_future.result.success, self.dock_svc_future.result.is_docked))
                    # if self.dock_svc_future.result.success == True:
                    if self.battery_state.charging == True:
                        self.last_dock_time = dt.datetime.now()
                        playtimeDurationInSeconds = (self.last_dock_time - self.last_undock_time).total_seconds()
                        playtimeDurationInDays = divmod(playtimeDurationInSeconds, 86400)
                        playtimeDurationInHours = round( (playtimeDurationInDays[1] / 3600.0), 1)
                        chargeCycles = int(daveDataJson.getData('chargeCycles')) + 1
                        daveDataJson.saveData('chargeCycles', chargeCycles)

                        if (playtimeDurationInHours > 0.1):
                            printMsg = "---- GoPi5Go-Dave ROS Docking {} : success at battery {:.0f}v after {:.1f} h playtime ".format(chargeCycles, self.docked_at_Vbatt, playtimeDurationInHours)
                            self.lifeLog.info(printMsg)
                            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                            print(dtstr, printMsg)
                            daveDataJson.saveData('lastDocking',printMsg)
                            daveDataJson.saveData('lastDockingTime', dtstr)
                            daveDataJson.saveData('lastPlaytimeDuration', playtimeDurationInHours)
                            daveDataJson.saveData('chargingState',"charging")
                        daveDataJson.saveData('dockingState',"docked")
                        self.prior_state = self.state
                        self.state = "docked"

            elif (self.state in ["docked"]):
                if (self.battery_state.milliamps > UNDOCK_AT_MILLIAMPS):
                    self.prior_state = self.state
                    self.state = "undocking"
                    self.last_undock_time = dt.datetime.now()
                    chargeDurationInSeconds = (self.last_undock_time - self.last_dock_time).total_seconds()
                    chargeDurationInDays = divmod(chargeDurationInSeconds, 86400)
                    chargeDurationInHours = round( (chargeDurationInDays[1] / 3600.0), 1)
                    if (chargeDurationInHours > 0.1):
                        printMsg = "---- GoPi5Go-Dave ROS 2 Undocking at Charge Current {:d} mA {:.2f}v after {:.1f} h charging".format(UNDOCK_AT_MILLIAMPS,self.battery_state.volts, chargeDurationInHours)
                        self.lifeLog.info(printMsg)
                        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        if DEBUG:
                            print(dtstr,printMsg)

                        daveDataJson.saveData('lastDismount', printMsg)
                        daveDataJson.saveData('lastRechargeDuration', chargeDurationInHours)
                        daveDataJson.saveData('lastDismountTime', dtstr)
                        daveDataJson.saveData('dockingState',"undocked")
                        daveDataJson.saveData('chargingState',"discharging")

                    if DEBUG:
                        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        printMsg = "dave_main_cb(): battery_status.milliamps {:.0f} mA, calling undock service ".format(self.battery_state.milliamps)
                        print(dtstr, printMsg)
                    # call undock service
                    if self.undock_svc_client.service_is_ready():
                        self.send_undock_svc_req()
                        self.prior_state = self.state
                        self.state = "undocking"
                else:       # docked and charging
                    if DEBUG:
                        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        printMsg = "dave_main_cb(): docked and charging: battery_status {:.1f}v {:.0f} mA {:.1f}W".format(self.battery_state.volts,self.battery_state.milliamps,self.battery_state.watts)
                        print(dtstr, printMsg)

            elif self.state == "undocking":
                if DEBUG:
                    printMsg = "dave_main_cb: exec undocking"
                    print(dtstr, printMsg)
                    print("self.undock_svc_future:", self.undock_svc_future.__dict__)
                if self.undock_svc_future._done == True:  # print("self.undock_svc_future:", self.undock_svc_future.__dict__)
                    if DEBUG:
                        print("undock_svc_future.result._success {}  is_docked: {}".format(self.undock_svc_future._result.success, self.undock_svc_future._result.is_docked))
                    if self.undock_svc_future._result.success == True:
                        self.prior_state = self.state
                        self.state = "ready_to_dock"
                """
                else:
                    if self.dock_status.is_docked == False:
                        self.state = "ready_to_dock"
                        if DEBUG:
                            printMsg = "dave_main_cb: undock_svc_future not done, but self.dock_status.is_docked is False - forcing 'ready_to_dock' state"
                            print(dtstr, printMsg)
                """
            elif (self.state in ["ready_to_dock"]):
                if (self.battery_state.volts < DOCK_AT_VOLTS):
                    self.prior_state = self.state
                    self.state = "docking"
                    self.docked_at_Vbatt = self.battery_state.volts
                    if DEBUG:
                        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        printMsg = "dave_main_cb(): battery_status.volts {:.1f}v calling dock service ".format(self.battery_state.volts)
                        print(dtstr, printMsg)
                    # call dock service
                    if self.dock_svc_client.service_is_ready():
                        self.send_dock_svc_req()
                else:  # playtime and no need to dock
                    if DEBUG:
                        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        printMsg = "dave_main_cb(): playtime: battery_status {:.1f}v {:.0f} mA {:.1f}W".format(self.battery_state.volts,self.battery_state.milliamps,self.battery_state.watts)
                        print(dtstr, printMsg)

            else:    # PROBLEM IF HERE
                printMsg = "dave_main_cb: exec else (self.state not a handled value)"
                print(dtstr, printMsg)


        except Exception as e:
            print("dave_main_cb: Exception:",str(e))
            traceback.print_exc(file=sys.stdout)
            sys.exit(1)


def main():
    rclpy.init(args=None)
    dave_node = DaveNode()
    mtexecutor = rclpy.executors.MultiThreadedExecutor(8)
    mtexecutor.add_node(dave_node)
    mtexecutor_thread = Thread(target=mtexecutor.spin, daemon=True)
    mtexecutor_thread.start()
    try:
        # rclpy.spin(test_node)
        while rclpy.ok():
           time.sleep(10)
    except KeyboardInterrupt:
        print("dave_node: main: KeyboardInterrupt")
        pass
    except ExternalShutdownException:
        print("dave_node: main: ExternalShutdownException")
        pass
    finally:
        print("dave_node: main: destroy_node")
        dave_node.destroy_node()
        try:
            print("dave_node: main: try_shutdown")
            rclpy.try_shutdown()
        except:
            print("dave_node: main: try_shutdown exception")
            pass
    # mtexecutor_thread.join()
    # rclpy.shutdown()



if __name__ == '__main__':
    main()



