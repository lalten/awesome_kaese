#! /usr/bin/python

import rospy
import cv2
import numpy as np
from tf2_msgs.msg import LookupTransformAction, LookupTransformGoal, LookupTransformResult
from actionlib import SimpleActionClient
from nav_msgs.msg import OccupancyGrid

class CleanZone:

    def __init__(self):
        self.map_img = None

        hector_ns = "/hector_mapping/"
        self.resolution = rospy.get_param(hector_ns+"map_resolution")
        self.cells_per_axis = rospy.get_param(hector_ns+"map_size")

        self.border_length = self.resolution*self.cells_per_axis
        rospy.loginfo("Using map size of %.2f m and resolution of %.2f m",
                      self.border_length, self.resolution)

        self.map_frame = rospy.get_param(hector_ns + "map_frame")
        self.base_frame = rospy.get_param(hector_ns + "base_frame")

        rospy.loginfo("Map frame: '%s', robot base frame: '%s'", self.map_frame, self.base_frame)

        self.tf_client = SimpleActionClient("/tf2_buffer_server", LookupTransformAction)
        if not self.tf_client.wait_for_server(rospy.Duration(2)):
            rospy.logerr("no tf2 available!")

        rospy.loginfo("starting cleaning map")
        self.start_clean_zone_map()

        self.update_timer = rospy.Timer(rospy.Duration(1), self.timer_cb)

        self.pub_clean_map = rospy.Publisher("clean_map", OccupancyGrid)

        og = OccupancyGrid()


        # self._metric_to_pixel(0)
        # self._metric_to_pixel(25)
        # self._metric_to_pixel(-25)

    def _get_position(self):
        goal = LookupTransformGoal()
        goal.source_frame = self.base_frame
        goal.target_frame = self.map_frame
        self.tf_client.send_goal(goal)
        if not self.tf_client.wait_for_result(rospy.Duration(3)):
            rospy.logerr("TF2 not available")
            return None

        res = self.tf_client.get_result()
        # print res

        assert isinstance(res, LookupTransformResult)
        if res.error.error != 0:
            rospy.logerr("TF Lookup failed: %s", res.error.error_string)
            return None

        return res.transform.transform.translation

    def timer_cb(self, timer_event):
        rospy.loginfo("Timer cb")
        pos = self._get_position()
        px_x = self._metric_to_pixel(pos.x)
        px_y = self._metric_to_pixel(pos.y)

        if not (px_x and px_y):
            return

        cv2.circle(self.map_img, (px_x, px_y), 30, 255, -1)
        cv2.imwrite("/tmp/map.png", self.map_img)

    def _metric_to_pixel(self, d):
        # assuming map_start_x/y == 0.5 (?)
        px = d/self.resolution + 0.5*self.cells_per_axis

        if not (0 < abs(px) < self.cells_per_axis):
            print d, px
            rospy.logwarn("Robot not in map at %.2f", d)
            return None

        return int(px)

    def start_clean_zone_map(self):
        rospy.loginfo("Creating new clean map with length %i", self.cells_per_axis)
        self.map_img = np.zeros((self.cells_per_axis, self.cells_per_axis, 1), np.uint8)
        # cv2.imwrite("/tmp/map.png", self.map_img)

        pos = self._get_position()
        if not pos:
            return


if __name__ == "__main__":
    rospy.init_node("clean_zone")
    cz = CleanZone()
    rospy.spin()