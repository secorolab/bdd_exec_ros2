#!/usr/bin/env python3

# Copyright 2026 Minh Nguyen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from bdd_ros2_interfaces.msg import Event


__DEFAULT_NODE_NAME = "mockup_behaviour"


class MockupBhvNode(Node):
    event_topic: str

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.declare_parameter("event_topic", "")

        use_sim_time = self.get_parameter("use_sim_time").value
        self.get_logger().info(f"use_sim_time: {use_sim_time}")

        self.event_topic = self.get_parameter("event_topic").value
        assert self.event_topic, f"{self.get_name()}: no 'event_topic' param specified"
        self.get_logger().info(f"Event topic: {self.event_topic}")

        self.evt_sub = self.create_subscription(
            msg_type=Event,
            topic=self.event_topic,
            callback=self.evt_sub_cb,
            qos_profile=10,
        )

    def evt_sub_cb(self, msg: Event):
        self.get_logger().info(f"{msg.stamp}: {msg.uri}")


def main(args=None):
    if args is None:
        node_name = __DEFAULT_NODE_NAME
    else:
        node_name = getattr(args, "node_name", __DEFAULT_NODE_NAME)

    try:
        with rclpy.init(args=args):
            mockup_bhv_node = MockupBhvNode(node_name=node_name)
            rclpy.spin(mockup_bhv_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
