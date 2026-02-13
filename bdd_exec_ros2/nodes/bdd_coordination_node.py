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
from bdd_dsl.models.urirefs import URI_BHV_PRED_TARGET_OBJ
from bdd_ros2_interfaces.action import Behaviour
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.executors import ExternalShutdownException
from bdd_ros2_interfaces.msg import Event, ParamValue
from rdflib import Namespace


TEST_NS = Namespace("https://secorolab.github.io/models/test/")
__DEFAULT_NODE_NAME = "test_coordinator"
# __SCR_START_EVT_URI = URIRef("https://my.url/models/evt_scr_start")


class BddCoordNode(Node):
    timeout_sec: float
    server_name: str
    _action_client: ActionClient
    _evt_pub: Publisher
    _evt_sub: Subscription

    def __init__(self, node_name: str, timeout_sec: float = 5.0) -> None:
        super().__init__(node_name)
        self.timeout_sec = timeout_sec

        self.declare_parameter("bhv_server_name", "bhv_server")
        self.declare_parameter("event_topic", "")

        use_sim_time = self.get_parameter("use_sim_time").value
        self.get_logger().info(f"use_sim_time: {use_sim_time}")

        # Behaviour action server
        self.server_name = self.get_parameter("bhv_server_name").value
        self.get_logger().info(f"Behaviour server name: {self.server_name}")
        self._action_client = ActionClient(self, Behaviour, self.server_name)

        # Topic for events
        self.event_topic = self.get_parameter("event_topic").value
        assert self.event_topic, f"{self.get_name()}: no 'event_topic' param specified"
        self.get_logger().info(f"Event topic: {self.event_topic}")

        self._evt_pub = self.create_publisher(
            msg_type=Event, topic=self.event_topic, qos_profile=10
        )

        self._evt_sub = self.create_subscription(
            msg_type=Event,
            topic=self.event_topic,
            callback=self.evt_sub_cb,
            qos_profile=10,
        )

    def evt_sub_cb(self, msg: Event):
        self.get_logger().info(f"{msg.stamp.sec}: {msg.uri}")
        goal_msg = Behaviour.Goal()
        param = ParamValue()
        param.param_rel_uri = URI_BHV_PRED_TARGET_OBJ.toPython()
        param.param_val_uris = [TEST_NS["cube"].toPython()]
        goal_msg.parameters = [param]
        self._action_client.wait_for_server(timeout_sec=self.timeout_sec)
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.bhv_feedback_cb
        )

        send_goal_future.add_done_callback(self.bhv_goal_resp_cb)

    def bhv_goal_resp_cb(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.bhv_result_cb)

    def bhv_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info("Behaviour feedback: {0}".format(feedback.status))

    def bhv_result_cb(self, future):
        result = future.result().result
        self.get_logger().info("Result: {0}".format(result.result))


def main(args=None):
    import rdf_utils

    print(rdf_utils.__path__)
    if args is None:
        node_name = __DEFAULT_NODE_NAME
    else:
        node_name = getattr(args, "node_name", __DEFAULT_NODE_NAME)

    try:
        with rclpy.init(args=args):
            mockup_bhv_node = BddCoordNode(node_name=node_name)
            rclpy.spin(mockup_bhv_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
