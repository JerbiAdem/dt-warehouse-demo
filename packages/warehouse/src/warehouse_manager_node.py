#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import AprilTagDetectionArray, WheelsCmdStamped

# Route definition
ROUTE = ['A', 'B', 'C', 'D']

# AprilTag ID → Station name mapping
STATION_MAP = {
    1: 'A',
    2: 'B',
    3: 'C',
    4: 'D',
}

# FSM States
STATE_NAVIGATING    = 'NAVIGATING'
STATE_ROTATING_BACK = 'ROTATING_BACK'
STATE_LOADING       = 'LOADING'
STATE_ROTATING_FWD  = 'ROTATING_FWD'
STATE_MISSION_COMPLETE = 'MISSION_COMPLETE'


class WarehouseManagerNode:

    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh = rospy.get_param('~veh', 'mybot')

        rospy.loginfo(f'[{self.node_name}] Vehicle name: {self.veh}')
        rospy.loginfo(f'[{self.node_name}] Subscribing to: /{self.veh}/apriltag_detector_node/detections')

        # FSM
        self.state = STATE_NAVIGATING
        self.current_target_idx = 0

        # Tunable parameters
        self.rotation_duration = rospy.get_param('~rotation_duration', 2.0)  # seconds for 180°
        self.loading_duration  = rospy.get_param('~loading_duration', 3.0)   # seconds at station
        self.rotation_speed    = rospy.get_param('~rotation_speed', 0.5)     # wheel speed for rotation

        # Publisher
        self.pub_wheels = rospy.Publisher(
            f'/{self.veh}/wheels_driver_node/wheels_cmd',
            WheelsCmdStamped,
            queue_size=1
        )

        # Subscriber
        rospy.Subscriber(
            f'/{self.veh}/apriltag_detector_node/detections',
            AprilTagDetectionArray,
            self.cb_apriltag,
            queue_size=1
        )

        rospy.loginfo(f'[{self.node_name}] Started. Route: {ROUTE}')
        rospy.loginfo(f'[{self.node_name}] First target: {ROUTE[self.current_target_idx]}')

    def cb_apriltag(self, msg):
        if self.state != STATE_NAVIGATING:
            return

        if self.current_target_idx >= len(ROUTE):
            return

        target_station = ROUTE[self.current_target_idx]

        for detection in msg.detections:
            station = STATION_MAP.get(detection.tag_id)
            if station == target_station:
                rospy.loginfo(f'[{self.node_name}] Station {station} detected — stopping!')
                self.stop_robot()
                self.state = STATE_ROTATING_BACK
                rospy.Timer(
                    rospy.Duration(0.5),  # small delay before rotating
                    lambda e: self.start_rotation_back(),
                    oneshot=True
                )
                break

    def start_rotation_back(self):
        rospy.loginfo(f'[{self.node_name}] Rotating back to station...')
        self.rotate(direction='left')
        rospy.Timer(
            rospy.Duration(self.rotation_duration),
            lambda e: self.start_loading(),
            oneshot=True
        )

    def start_loading(self):
        rospy.loginfo(f'[{self.node_name}] Loading at station {ROUTE[self.current_target_idx]}...')
        self.stop_robot()
        self.state = STATE_LOADING
        rospy.Timer(
            rospy.Duration(self.loading_duration),
            lambda e: self.start_rotation_fwd(),
            oneshot=True
        )

    def start_rotation_fwd(self):
        rospy.loginfo(f'[{self.node_name}] Rotating back to forward position...')
        self.state = STATE_ROTATING_FWD
        self.rotate(direction='left')  # same direction = completes the 360° total
        rospy.Timer(
            rospy.Duration(self.rotation_duration),
            lambda e: self.advance_to_next_station(),
            oneshot=True
        )

    def advance_to_next_station(self):
        self.stop_robot()
        self.current_target_idx += 1

        if self.current_target_idx >= len(ROUTE):
            self.state = STATE_MISSION_COMPLETE
            rospy.loginfo(f'[{self.node_name}] Mission complete!')
            return

        self.state = STATE_NAVIGATING
        rospy.loginfo(f'[{self.node_name}] Next target: {ROUTE[self.current_target_idx]}')

    def rotate(self, direction='left'):
        msg = WheelsCmdStamped()
        if direction == 'left':
            msg.vel_left  = -self.rotation_speed
            msg.vel_right =  self.rotation_speed
        else:
            msg.vel_left  =  self.rotation_speed
            msg.vel_right = -self.rotation_speed
        self.pub_wheels.publish(msg)

    def stop_robot(self):
        msg = WheelsCmdStamped()
        msg.vel_left  = 0.0
        msg.vel_right = 0.0
        self.pub_wheels.publish(msg)


if __name__ == '__main__':
    rospy.init_node('warehouse_manager_node', anonymous=False)
    node = WarehouseManagerNode()
    rospy.spin()
