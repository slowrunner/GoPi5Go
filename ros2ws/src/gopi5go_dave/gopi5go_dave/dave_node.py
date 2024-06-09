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

from dave_interfaces.srv import Dock, Undock
from dave_interfaces.msg import BatteryState, DockStatus

import sys
sys.path.insert(1,'/home/pi/GoPi5Go/plib')
import daveDataJson

import traceback
import logging
import datetime as dt

DEBUG = False
# uncomment following for debug 
DEBUG = True


LIFELOGFILE = "/home/pi/GoPi5Go/logs/life.log"


UNDOCK_AT_MILLIAMPS =  -175
DOCK_AT_VOLTS       =   10

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
        printMsg = '** GoPi5Go-Dave node started - Undock:{:d} mA  Dock:{:.2f} v **'.format(int(UNDOCK_AT_MILLIAMPS), int(DOCK_AT_VOLTS) )
        print(dtstr,printMsg)
        self.lifeLog.info(printMsg)

        self.battery_state_sub = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_state_cb,
            qos_profile_sensor_data)    # best effort depth 10 

        self.battery_state = BatteryState()

        self.sub = self.create_subscription(
            DockStatus,
            'dock_status',
            self.dock_status_cb,
            qos_profile_sensor_data)    # best effort depth 10 

        self.dock_status = DockStatus()

        self.state = "init"
        # action clients

        # service clients
        self.dock_client = self.create_client(Dock, 'dock')
        self.dock_request = Dock.Request()

        self.undock_client = self.create_client(Undock, 'undock')
        self.undock_request = Undock.Request()

        self.hz = 1   # execute dave_cb once every second
        self.timer = self.create_timer(1.0/self.hz, self.dave_main_cb)   # call dave_main_cb once every period

        self.state = "init"
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
            printMsg = "battery_state_cb(): battery_state.charging {} ".format(self.battery_state.charging)
            print(dtstr,printMsg)

    def dock_status_cb(self,dock_status_msg):
        self.dock_status = dock_status_msg
        self.all_req_topic_cb_rx[1] = True  # /battery_state, /dock_status
        if DEBUG:
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg = "dock_status_cb(): dock_status.is_docked {} ".format(self.dock_status.is_docked)
            print(dtstr,printMsg)

    def request_dock(self):
        self.dock_future = self.dock_client.call_async(self.dock_request)

    def 


    def dave_main_cb(self):
        try:

            # evaluate dave.state validity
            print("Entering dave_main_cb:")
            if (self.state == "init"):       # Handle startup - wait for subcriptions, then figure out if docked or undocked
                if (self.all_req_topic_cb_rx[0] == False) or (self.all_req_topic_cb_rx[1] == False):
                  if DEBUG:
                    dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    printMsg = "dave_main_cb(): waiting for all_req_topic_cb_rx"
                    print(dtstr, printMsg)
                    printMsg = "                battery_state, dock_status = {}".format(self.all_req_topic_cb_rx)
                    print(dtstr, printMsg)
                elif self.battery_state.charging:     # init and all subcribers have received topics
                    self.state = "docked"
                else:                              # PROBABLY UNDOCKED - Cannot be certain from only charging status
                    self.state = "undocked"

            elif (self.dock_status.is_docked):    # Handle docked and "docked but don't know it yet"
                print("dave_main_cb: docked state detected")
                if (self.state != "docked"):
                    self.last_dock_time = dt.datetime.now()
                    playtimeDurationInSeconds = (self.last_dock_time - self.last_undock_time).total_seconds()
                    playtimeDurationInDays = divmod(playtimeDurationInSeconds, 86400)
                    playtimeDurationInHours = round( (playtimeDurationInDays[1] / 3600.0), 1)
                    if (playtimeDurationInHours > 0.1):
                        printMsg = "** Dave Noticed Docking: success at battery {:.0f}v after {:.1f} hrs playtime **".format(self.battery_state.volts, playtimeDurationInHours)
                        self.lifeLog.info(printMsg)
                        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        print(dtstr, printMsg)
                    self.state = "docked"

                else:     # handle docked behavior
                    if DEBUG:
                        printMsg = "charging: battery_state: {:.0f} mA {:.2f} v".format(self.battery_state.milliamps, self.battery_state.volts)
                        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        print(dtstr, printMsg)

            elif (self.state == "docked"):
                print("dave_main_cb: not docked but think docked")
                self.state = "undocked"
                self.last_undock_time = dt.datetime.now()
                chargeDurationInSeconds = (self.last_undock_time - self.last_dock_time).total_seconds()
                chargeDurationInDays = divmod(chargeDurationInSeconds, 86400)
                chargeDurationInHours = round( (chargeDurationInDays[1] / 3600.0), 1)
                if (chargeDurationInHours > 0.1):
                    printMsg = "** Dave Noticed Undocking at battery {:.0f}%, docked for {:.1f} hrs **".format(self.battery_state.percent, chargeDurationInHours)
                    self.lifeLog.info(printMsg)
                    if DEBUG:
                        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        print(dtstr,printMsg)
            else:
                print("dave_main_cb: setting state = undocked")
                self.state = "undocked"

            if DEBUG:
                dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                printMsg = "dave_main_cb(): executing"
                print(dtstr, printMsg)
                printMsg = "dave_main_cb(): dave.state = {}".format(self.state)
                print(dtstr, printMsg)

            if (self.state in ["docked"]):
                if (self.battery_state.milliamps > UNDOCK_AT_MILLIAMPS):
                    self.state = "undocking"
                    self.last_undock_time = dt.datetime.now()
                    chargeDurationInSeconds = (self.last_undock_time - self.last_dock_time).total_seconds()
                    chargeDurationInDays = divmod(chargeDurationInSeconds, 86400)
                    chargeDurationInHours = round( (chargeDurationInDays[1] / 3600.0), 1)
                    if (chargeDurationInHours > 0.1):
                        printMsg = "** Dave Noticed Undocking at battery {:.0f}%, docked for {:.1f} hrs **".format(self.battery_state.percent, chargeDurationInHours)
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
                else:       # docked and charging
                    if DEBUG:
                        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        printMsg = "dave_main_cb(): docked and charging: battery_status {:.1f}v {:.0f} mA".format(self.battery_state.volts,self.battery_state.milliamps)
                        print(dtstr, printMsg)

            elif (self.state in ["undocked"]):
                if (self.battery_state.volts < DOCK_AT_VOLTS):
                    self.state = "docking"
                    if DEBUG:
                        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        printMsg = "dave_main_cb(): battery_status.volts {:.1f}v calling dock service ".format(self.battery_state.volts)
                        print(dtstr, printMsg)
                        # call undock service
                else:
                    if DEBUG:
                        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        printMsg = "dave_main_cb(): playtime: battery_status {:.1f}v {:.0f} mA".format(self.battery_state.volts,self.battery_state.milliamps)
                        print(dtstr, printMsg)

        except Exception as e:
            print("dave_main_cb: Exception:",str(e))
            traceback.print_exc(file=sys.stdout)
            sys.exit(1)


def main():
    rclpy.init(args=None)
    dave_node = DaveNode()
    try:
        rclpy.spin(dave_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(0)
    finally:
        dave_node.destroy_node()
        try:
            rclpy.try_shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
