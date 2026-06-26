#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from duckietown_msgs.msg import AprilTagDetectionArray, BoolStamped, WheelsCmdStamped


# Route definition: list of station names in order
ROUTE = ['A', 'B', 'C', 'D']

# AprilTag ID → Station name mapping
STATION_MAP = {
    1: 'A',
    2: 'B',
    3: 'C',
    4: 'D',
}

# FSM States
STATE_NAVIGATING = 'NAVIGATING'
STATE_AT_STATION = 'AT_STATION'
STATE_OBSTACLE_STOPPED = 'OBSTACLE_STOPPED'
STATE_MISSION_COMPLETE = 'MISSION_COMPLETE'


class WarehouseManagerNode:

    def __init__(self):
        self.node_name = rospy.get_name()

        # Mission state
        self.state = STATE_NAVIGATING
        self.current_target_idx = 0
        self.obstacle_detected = False
        self.at_stop_line = False
        self.station_stop_duration = rospy.get_param("~station_stop_duration", 3.0)

        # Publishers
        self.pub_wheels_cmd = rospy.Publisher(
            f"/{rospy.get_param('~veh')}/wheels_driver_node/wheels_cmd",
            WheelsCmdStamped,
            queue_size=1
        )

        # Subscribers
        self.sub_obstacle = rospy.Subscriber(
            "~obstacle_detected",
            Bool,
            self.cb_obstacle,
            queue_size=1
        )

        self.sub_apriltag = rospy.Subscriber(
            f"/{rospy.get_param('~veh')}/apriltag_detector_node/detections",
            AprilTagDetectionArray,
            self.cb_apriltag,
            queue_size=1
        )

        self.sub_stop_line = rospy.Subscriber(
            f"/{rospy.get_param('~veh')}/stop_line_filter_node/at_stop_line",
            BoolStamped,
            self.cb_stop_line,
            queue_size=1
        )

        rospy.loginfo(f"[{self.node_name}] Initialized. Route: {ROUTE}")
        rospy.loginfo(f"[{self.node_name}] First target: {ROUTE[self.current_target_idx]}")

    def cb_obstacle(self, msg):
        self.obstacle_detected = msg.data

        if self.obstacle_detected and self.state == STATE_NAVIGATING:
            self.state = STATE_OBSTACLE_STOPPED
            self.stop_robot()
            rospy.loginfo(f"[{self.node_name}] OBSTACLE DETECTED — Stopping!")

        elif not self.obstacle_detected and self.state == STATE_OBSTACLE_STOPPED:
            self.state = STATE_NAVIGATING
            rospy.loginfo(f"[{self.node_name}] Path clear — Resuming!")

    def cb_apriltag(self, msg):
        if self.state != STATE_NAVIGATING:
            return

        target_station = ROUTE[self.current_target_idx]

        for detection in msg.detections:
            station = STATION_MAP.get(detection.tag_id)
            if station == target_station and self.at_stop_line:
                rospy.loginfo(f"[{self.node_name}] Arrived at station {station}!")
                self.state = STATE_AT_STATION
                self.stop_robot()
                rospy.Timer(
                    rospy.Duration(self.station_stop_duration),
                    self.advance_to_next_station,
                    oneshot=True
                )
                break

    def cb_stop_line(self, msg):
        self.at_stop_line = msg.data

    def stop_robot(self):
        msg = WheelsCmdStamped()
        msg.vel_left = 0.0
        msg.vel_right = 0.0
        self.pub_wheels_cmd.publish(msg)

    def advance_to_next_station(self, event):
        self.current_target_idx = (self.current_target_idx + 1) % len(ROUTE)
        self.at_stop_line = False
        self.state = STATE_NAVIGATING
        rospy.loginfo(f"[{self.node_name}] Next target: {ROUTE[self.current_target_idx]}")


if __name__ == "__main__":
    rospy.init_node("warehouse_manager_node", anonymous=False)
    node = WarehouseManagerNode()
    rospy.spin()