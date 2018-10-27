#! /usr/bin/python

import rospy
import cv2
import numpy as np
from tf2_msgs.msg import LookupTransformAction, LookupTransformGoal, LookupTransformResult, TFMessage
from actionlib import SimpleActionClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# This class
class CleanZone:

    def __init__(self):
        self.map_img = None
        self.cpy_img = None

        hector_ns = "/hector_mapping/"
        self.resolution = rospy.get_param(hector_ns+"map_resolution")
        self.cells_per_axis = int(rospy.get_param(hector_ns+"map_size")*0.55)  # just for video

        self.border_length = self.resolution*self.cells_per_axis
        rospy.loginfo("Using map size of %.2f m and resolution of %.2f m",
                      self.border_length, self.resolution)

        self.map_frame = rospy.get_param(hector_ns + "map_frame")
        self.base_frame = rospy.get_param(hector_ns + "base_frame")

        self.pub_img = rospy.Publisher("clean_img", Image, queue_size=1)

        rospy.loginfo("Map frame: '%s', robot base frame: '%s'", self.map_frame, self.base_frame)

        rospy.loginfo("starting cleaning map")
        self.start_clean_zone_map()
        self.bridge = CvBridge()

        self.use_tf2_action = False

        if self.use_tf2_action:
            self.tf_client = SimpleActionClient("/tf2_buffer_server", LookupTransformAction)
            if not self.tf_client.wait_for_server(rospy.Duration(2)):
                rospy.logerr("no tf2 available!")

            self.update_timer = rospy.Timer(rospy.Duration(1), self.timer_cb)
        else:
            self.pub_tf = rospy.Subscriber("/tf", TFMessage, self.tf_cb, queue_size=1)
            self.last_tf_time = 0
            self.diff_tf_time = rospy.Duration(0.5)

    def _get_position(self):
        goal = LookupTransformGoal()
        goal.source_frame = self.base_frame
        goal.target_frame = self.map_frame
        self.tf_client.send_goal(goal)
        if not self.tf_client.wait_for_result(rospy.Duration(3)):
            rospy.logerr("TF2 not available")
            return None

        res = self.tf_client.get_result()

        assert isinstance(res, LookupTransformResult)
        if res.error.error != 0:
            rospy.logerr("TF Lookup failed: %s", res.error.error_string)
            return None

        return res.transform.transform.translation

    def tf_cb(self, tfm):
        assert isinstance(tfm, TFMessage)
        for t in tfm.transforms:
            if t.header.frame_id == self.map_frame and t.child_frame_id == self.base_frame:
                if self.last_tf_time == 0 or self.last_tf_time + self.diff_tf_time < t.header.stamp:
                    self.draw_at_position(t.transform.translation)
                    self.last_tf_time = t.header.stamp

    def draw_at_position(self, position):
        px_x = self._metric_to_pixel(position.x)
        px_y = self._metric_to_pixel(position.y)

        if not (px_x and px_y):
            return

        px_y = self.cells_per_axis - px_y

        radius = 20
        new_col = 30

        cv2.circle(self.cpy_img, (px_x, px_y), radius, new_col, -1)
        cv2.add(self.map_img, self.cpy_img, self.map_img)

        cv2.circle(self.cpy_img, (px_x, px_y), radius, 0, -1)

        cv_image = self.bridge.cv2_to_imgmsg(self.map_img, encoding="mono8")
        self.pub_img.publish(cv_image)

        cv2.imwrite("/tmp/map.png", self.map_img)

    def timer_cb(self, _):
        pos = self._get_position()
        if not pos:
            return
        self.draw_at_position(pos)

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
        self.cpy_img = np.zeros((self.cells_per_axis, self.cells_per_axis, 1), np.uint8)

if __name__ == "__main__":
    rospy.init_node("clean_zone")
    cz = CleanZone()
    rospy.spin()