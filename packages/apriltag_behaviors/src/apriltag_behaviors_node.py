#!/usr/bin/env python3

import json
from typing import Dict

import rospy

from duckietown.dtros import DTROS, NodeType

from duckietown_msgs.msg import AprilTagDetectionArray, BoolStamped, LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern


TagID = int


class AprilTagBehavior(DTROS):
    def __init__(self):
        super(AprilTagBehavior, self).__init__(
            node_name="apriltag_behaviors_node", node_type=NodeType.PERCEPTION
        )
        self._veh = rospy.get_param("~veh")
        # get static parameters
        behaviors = rospy.get_param("~behaviors", [])
        self._behaviors: Dict[TagID, dict] = {b["tag"]: b for b in behaviors}
        # create subscribers
        self._sub = rospy.Subscriber(
            "~detections", AprilTagDetectionArray, self._cb, queue_size=1
        )
        # create publishers
        # self.pub_joy_override = rospy.Publisher(
        #     "~joystick_override", BoolStamped, queue_size=1
        # )
        self.pub_estop = rospy.Publisher(
            "~estop", BoolStamped, queue_size=1
        )
        self.changePattern = rospy.ServiceProxy("~set_custom_pattern", SetCustomLEDPattern)
        # internal state
        self._last_behavior = None
        # print out behaviors
        self.loginfo(f"Behaviors are: {json.dumps(self._behaviors, indent=4, sort_keys=True)}")

    def led_patterns(self, lf: bool) -> LEDPattern:
        msg = LEDPattern()

        # DB21+/4-LED bot LED mapping
        # 0 - front left
        # 2 - front right
        # 3 - rear right
        # 4 - rear left

        color_list = ["white", "white", "white", "red", "red"]

        msg.color_list = color_list
        msg.color_mask = []

        # if stopped:
        #     msg.frequency = 5.0
        #     msg.frequency_mask = [0, 0, 0, 1, 1]
        # elif detection:
        #     msg.frequency = 1.0
        #     msg.frequency_mask = [0, 0, 0, 1, 1]
        # else:
        #     # no blinking for normal lane_following
        #     # ref: if self.state == "LANE_FOLLOWING":
        msg.frequency = 0.0
        msg.frequency_mask = [0]
        if not lf:
            msg.color_list = ["white", "white", "white", "white", "white"]

        return msg

    def _cb(self, msg):
        for detection in msg.detections:
            new_behavior = self._behaviors.get(detection.tag_id, None)
            if new_behavior is not None and new_behavior != self._last_behavior:
                self.loginfo(f"Switched to new behavior: {new_behavior['name']}")
                # publish message
                # Back button: Stop LF

                behavior_name = new_behavior["name"]

                if behavior_name in {"LANE_FOLLOWING_MODE", "ESTOP"}:
                    in_lf_mode = behavior_name == "LANE_FOLLOWING_MODE"

                    # override_msg = BoolStamped()
                    # override_msg.header.stamp = msg.header.stamp
                    # override_msg.data = False
                    # self.pub_joy_override.publish(override_msg)

                    estop_msg = BoolStamped()
                    estop_msg.header.stamp = msg.header.stamp
                    estop_msg.data = (not in_lf_mode)
                    self.pub_estop.publish(estop_msg)

                    self.changePattern(self.led_patterns(lf=in_lf_mode))

                if behavior_name in {"LEFT_HAND_TRAFFIC", "RIGHT_HAND_TRAFFIC"}:
                    traffic_mode = "LHT" if behavior_name == "LEFT_HAND_TRAFFIC" else "RHT"
                    rospy.set_param(f"/{self._veh}/behavior/traffic_mode", traffic_mode)

                self._last_behavior = new_behavior


if __name__ == "__main__":
    node = AprilTagBehavior()
    # spin forever
    rospy.spin()
