#!/usr/bin/python

import numpy as np
import math

import roslib
import rospy
import tf
import actionlib

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from homer_mapnav_msgs.msg import DetectObstacleAction
from homer_mapnav_msgs.msg import DetectObstacleActionResult
from homer_mapnav_msgs.msg import DetectObstacleResult

from visualization_msgs.msg import Marker

global_frame = "/map"
debug = False
debug_laser_transform = False

def _pub_marker(points, marker_id, r, g, b, a = 1, size = 0.04, marker_type =
                Marker.POINTS):
    if len(points) == 0:
        rospy.logwarn("No markers to publish for id {}.".format(marker_id))
        return
    markers = Marker()
    markers.header.frame_id = points[0].header.frame_id
    markers.header.stamp = points[0].header.stamp
    markers.ns = "/detect_obstacle"
    markers.action = Marker.ADD
    markers.pose.orientation.w = 1.0
    markers.id = marker_id
    markers.type = marker_type
    markers.scale.x = size
    markers.scale.y = size
    markers.color.a = a
    markers.color.r = r
    markers.color.g = g
    markers.color.b = b
    for point in points:
        markers.points.append(point.point)

    _marker_pub.publish(markers)


def _laser_msg_to_points(scan, tf_listener, to_frame, debug_id = 0):
    points = []
    time = rospy.Time(0)
    if ( not tf_listener.canTransform(scan.header.frame_id, to_frame, time)):
        rospy.logerr("[detect_obstacle_action] No transform found.")
        return points

    alpha = scan.angle_min
    for r in scan.ranges:
        if not (r < scan.range_min or r > scan.range_max):
            point = PointStamped()

            point.point.x = np.cos(alpha) * r
            point.point.y = np.sin(alpha) * r

            if math.isnan(point.point.x) or math.isnan(point.point.y):
              continue

            point.header.frame_id = scan.header.frame_id
            point.header.stamp = scan.header.stamp
            point.header.stamp = time
            points.append(tf_listener.transformPoint(to_frame, point))

        alpha += scan.angle_increment

    if debug_laser_transform:
        _pub_marker(points, debug_id, 0, debug_id, 1)

    return points


class DetectObstacle():

    def __init__(self):
        self.action = actionlib.SimpleActionServer("/detect_obstacle", 
                                                   DetectObstacleAction, 
                                                   self.detect_obstacle, False)
        self._tf_listener = tf.TransformListener()
        self._obstacle_radius = rospy.get_param(
            "/homer_navigation/allowed_obstacle_distance/min")

    def detect_obstacle(self, goal):
        rospy.loginfo("transforming point")
        location = self._tf_listener.transformPoint(global_frame, goal.location)
        if debug:
            _pub_marker([location], 4, 1, 1, 1, 0.5, self._obstacle_radius*2, 
                        Marker.SPHERE_LIST)

        rospy.loginfo("waiting for main scan...")
        main_scan = rospy.wait_for_message("/scan", LaserScan, 2.0)

        rospy.loginfo("waiting for front scan...")
        try:
            front_scan = rospy.wait_for_message("/front_scan", LaserScan, 2.0)
        except Exception as e:
            rospy.logerr(e.message)
            result = DetectObstacleResult()
            result.result = DetectObstacleResult.BIG_OBSTACLE
            self.action.set_succeeded(result)
            return

        result = self._detect_on_location(main_scan, front_scan, location)
        self.action.set_succeeded(result)

    def _detect_on_location(self, main_scan, front_scan, location):
        main_points = _laser_msg_to_points(main_scan, self._tf_listener,
                                           global_frame, 0)
        front_points = _laser_msg_to_points(front_scan, self._tf_listener,
                global_frame, 1)

        obstacle_points = self._find_obstacle_points(main_points, location)
        obstacle_points.extend(self._find_obstacle_points(front_points, 
                                                          location))

        if debug:
            _pub_marker(obstacle_points, 100, 1, 0, 0)

        mean = self._mean(obstacle_points)

        if debug:
            _pub_marker([mean], 101, 0, 1, 0, 0.5, self._obstacle_radius*2,
                        Marker.SPHERE_LIST)

        main_obstacle_points = self._find_obstacle_points(main_points, mean)
        front_obstacle_points = self._find_obstacle_points(front_points, mean)

        if debug:
            _pub_marker(main_obstacle_points, 102, 0, 0, 1)
            _pub_marker(front_obstacle_points, 103, 0, 0, 1)
            rospy.loginfo("Number of obstacle points in main scan: {}"
                          .format(len(main_obstacle_points)))
            rospy.loginfo("Number of obstacle points in front scan: {}"
                          .format(len(front_obstacle_points)))

        obstacle_in_main_scan = len(main_obstacle_points) > 5
        obstacle_in_front_scan = len(front_obstacle_points) > 5

        result = DetectObstacleResult()

        rospy.loginfo("Obstacle in main scan: {}"
                      .format(obstacle_in_main_scan))
        rospy.loginfo("Obstacle in front scan: {}"
                      .format(obstacle_in_front_scan))

        if obstacle_in_main_scan:
            if obstacle_in_front_scan:
                result.result = DetectObstacleResult.BIG_OBSTACLE
            else:
                result.result = DetectObstacleResult.SMALL_OBSTACLE
        else:
            if obstacle_in_front_scan:
                result.result = DetectObstacleResult.SMALL_OBSTACLE
            else:
                result.result = DetectObstacleResult.NO_OBSTACLE

        return result

    def _mean(self, stamped_points):
        mean = np.mean(
            [[p.point.x, p.point.y, p.point.z] for p in stamped_points], 
            axis=0)
        mean_point = PointStamped()
        mean_point.point.x = mean[0]
        mean_point.point.y = mean[1]
        mean_point.point.z = mean[2]
        mean_point.header.frame_id = stamped_points[0].header.frame_id
        mean_point.header.stamp = stamped_points[0].header.stamp
        return mean_point

    def _find_obstacle_points(self, points, mean_obstacle):
        obstacle_points = filter(
            lambda p: self._distance(p.point, mean_obstacle.point) 
            <= self._obstacle_radius, points)
        return obstacle_points

    def _distance(self, a, b):
        return np.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)


if __name__ == '__main__':
    rospy.init_node("detect_obstacle")

    node = DetectObstacle()
    node.action.start()

    if debug or debug_laser_transform:
        _marker_pub = rospy.Publisher("/visualization_marker", Marker,
                                      queue_size=10)

    rospy.sleep(1.0)
    rospy.loginfo("detect obstacle node loaded")
    rospy.spin()
