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

import os
import unittest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing
import launch_testing.actions

import pytest
import rclpy
from rclpy.action import ActionClient

from ament_index_python.packages import get_package_share_directory

from bdd_dsl.models.urirefs import URI_BHV_PRED_TARGET_OBJ
from bdd_ros2_interfaces.action import Behaviour
from bdd_ros2_interfaces.msg import ParamValue, Trinary
from rdflib import Namespace


TEST_NS = Namespace("https://secorolab.github.io/models/test/")


@pytest.mark.launch_test
def generate_test_description():
    pkg_share = get_package_share_directory("bdd_exec_ros2")

    launch_file = os.path.join(pkg_share, "launch", "launch_mockup.yaml")

    included_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(launch_file),
        launch_arguments={
            "ns": "bdd_test",
            "config": os.path.join(
                get_package_share_directory("bdd_exec_ros2"),
                "config",
                "mockup_configs_test.yaml",
            ),
        }.items(),
    )

    return (
        LaunchDescription(
            [
                included_launch,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestBehaviourActionServer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_behaviour_action_client")
        self.client = ActionClient(self.node, Behaviour, "/bdd_test/bhv_server")

        self.assertTrue(
            self.client.wait_for_server(timeout_sec=5.0), "Action server not available"
        )

    def tearDown(self):
        self.node.destroy_node()

    def test_goal_succeeds(self):
        # Create goal
        goal_msg = Behaviour.Goal()

        param = ParamValue()
        param._param_rel_uri = URI_BHV_PRED_TARGET_OBJ.toPython()
        param.param_val_uris = [TEST_NS["cube"].toPython()]
        goal_msg.parameters = [param]

        # Send goal
        send_goal_future = self.client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()

        self.assertIsNotNone(goal_handle)
        self.assertTrue(goal_handle.accepted, "Goal was rejected")

        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, get_result_future)

        result = get_result_future.result().result

        # ----------------------------------------------------------------------
        # Assertions
        # ----------------------------------------------------------------------
        self.assertEqual(
            result.result.value,
            Trinary.TRUE,
            msg=f"unexpected result value: {result.result.value} != {Trinary.TRUE}",
        )
