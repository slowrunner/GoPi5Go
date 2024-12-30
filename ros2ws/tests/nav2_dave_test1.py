#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
from math import pi

DEG_TO_RAD = pi/180.0

# POSITIONS [x,y,heading] relative to map
DOCK_POSITION = [6.843, -0.282, -180.0 * DEG_TO_RAD]
READY_POSITION = [6.672, -0.282, -180.0 * DEG_TO_RAD]
WALL_CORNER_POSITION = [6.750, 0.650, 90 * DEG_TO_RAD]
BTWN_WALL_SOFA_POSITION = [6.550, 1.550, 45 * DEG_TO_RAD]

def create_pose_stamped(navigator: BasicNavigator, position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

def main():
    # --- Init
    rclpy.init()
    nav = BasicNavigator()

    # --- Set initial pose
    initial_pose = create_pose_stamped(nav, READY_POSITION[0], READY_POSITION[1], READY_POSITION[2])
    nav.setInitialPose(initial_pose)

    # --- Wait for Nav2
    nav.waitUntilNav2Active()

    # --- Send Nav2 goal
    waypoints = []
    waypoints.append(create_pose_stamped(nav, WALL_CORNER_POSITION[0], WALL_CORNER_POSITION[1], WALL_CORNER_POSITION[2]))
    waypoints.append(create_pose_stamped(nav, BTWN_WALL_SOFA_POSITION[0], BTWN_WALL_SOFA_POSITION[1], BTWN_WALL_SOFA_POSITION[2]))

    # --- Follow waypoints
    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)

    print(nav.getResult())

    # --- Go to one pose
    # dock_ready = create_pose_stamped(nav, READY_POSITION[0], READY_POSITION[1], READY_POSITION[2])
    # nav.goToPose(dock_ready)
    # while not nav.isTaskComplete():
    #     feedback = nav.getFeedback()
    #     # print(feedback)



    # --- Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()


