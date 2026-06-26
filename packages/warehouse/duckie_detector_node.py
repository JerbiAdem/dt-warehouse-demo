#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from cv_bridge import CvBridge


class DuckieDetectorNode:

    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()

        # HSV bounds for duckie detection (calibrated during Braitenberg LX)
        self.lower_hsv = np.array([5, 80, 100])
        self.upper_hsv = np.array([25, 200, 255])

        # Minimum number of yellow pixels to trigger obstacle alert
        self.pixel_threshold = 5000

        # Publisher
        self.pub_obstacle = rospy.Publisher(
            "~obstacle_detected",
            Bool,
            queue_size=1
        )

        # Subscriber
        self.sub_image = rospy.Subscriber(
            "~image/compressed",
            CompressedImage,
            self.cb_image,
            queue_size=1,
            buff_size=2**24
        )

        rospy.loginfo(f"[{self.node_name}] Initialized.")

    def cb_image(self, msg):
        # Decode image
        img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Only look at bottom half of image (road level)
        h = img.shape[0]
        img_cropped = img[h // 2:, :]

        # Convert to HSV and apply color filter
        hsv = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)

        # Count yellow pixels
        yellow_pixels = np.sum(mask > 0)

        # Publish obstacle alert
        obstacle = bool(yellow_pixels > self.pixel_threshold)
        self.pub_obstacle.publish(Bool(data=obstacle))

        if obstacle:
            rospy.loginfo(f"[{self.node_name}] Duckie detected! ({yellow_pixels} pixels)")


if __name__ == "__main__":
    rospy.init_node("duckie_detector_node", anonymous=False)
    node = DuckieDetectorNode()
    rospy.spin()