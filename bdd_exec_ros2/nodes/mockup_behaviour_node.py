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
import time
from random import random
from rdflib import URIRef
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException
from coord_dsl.fsm import FSMData, fsm_step, produce_event
from coord_dsl.event_loop import reconfig_event_buffers
from bdd_ros2_interfaces.msg import Event, Trinary
from bdd_ros2_interfaces.action import Behaviour
from bdd_exec_ros2.behaviours.fsm_pickplace import EventID, StateID, create_fsm


__DEFAULT_NODE_NAME = "mockup_behaviour"
__SCR_START_EVT_URI = URIRef("https://my.url/models/evt_scr_start")


def random_in_range(lower, upper):
    return random() * (upper - lower) + lower


class UserData:
    start_time: float
    perceive_delay: float
    approach_delay: float
    pick_delay: float
    place_delay: float
    picking: bool
    placing: bool
    succeeded: Trinary

    def __init__(self, delay_lower=2.0, delay_upper=4.0) -> None:
        self.start_time = time.time()
        self.perceive_delay = random_in_range(delay_lower, delay_upper)
        self.approach_delay = random_in_range(delay_lower, delay_upper)
        self.pick_delay = random_in_range(delay_lower, delay_upper)
        self.place_delay = random_in_range(delay_lower, delay_upper)
        self.picking = False
        self.placing = False
        self.succeeded = Trinary()
        self.succeeded.value = Trinary.UNKNOWN

    def elapsed(self, cur_state: StateID) -> bool:
        if cur_state == StateID.S_PERCEIVE:
            delay = self.perceive_delay
        elif cur_state == StateID.S_APPROACH:
            delay = self.approach_delay
        elif cur_state == StateID.S_PICK:
            delay = self.pick_delay
        elif cur_state == StateID.S_PLACE:
            delay = self.place_delay
        else:
            raise ValueError(f"UserData.elapsed: unhandled state '{cur_state.name}'")

        cur_time = time.time()
        elapsed = self.start_time + delay < cur_time
        if elapsed:
            self.start_time = cur_time
        return elapsed


def fsm_mockup_bhv(fsm: FSMData, ud: UserData):
    current_state = StateID(fsm.current_state_index)

    if current_state == StateID.S_PERCEIVE:
        if not ud.elapsed(cur_state=current_state):
            return
        ud.picking = True
        produce_event(event_data=fsm.event_data, event_index=EventID.E_PERCEIVE_DONE)
        return

    if current_state == StateID.S_APPROACH:
        if not ud.elapsed(cur_state=current_state):
            return

        if ud.picking:
            assert not ud.placing, "both 'picking' & 'placing' are true in UserData"
            produce_event(fsm.event_data, event_index=EventID.E_PICK_APPROACH_DONE)
            return

        if ud.placing:
            assert not ud.picking, "both 'placing' & 'picking' are true in UserData"
            produce_event(fsm.event_data, event_index=EventID.E_PLACE_APPROACH_DONE)
            return

        raise AssertionError("Neither 'picking' or placing is true in approach state")

    if current_state == StateID.S_PICK:
        if not ud.elapsed(cur_state=current_state):
            return

        ud.picking = False
        ud.placing = True
        produce_event(fsm.event_data, event_index=EventID.E_PICK_DONE)
        return

    if current_state == StateID.S_PLACE:
        if not ud.elapsed(cur_state=current_state):
            return

        ud.placing = False
        ud.succeeded.value = Trinary.TRUE
        produce_event(fsm.event_data, event_index=EventID.E_PLACE_DONE)
        return


class MockupBhvNode(Node):
    event_topic: str
    loop_duration: float
    heartbeat_duration: float
    server_name: str
    _action_server: ActionServer

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.declare_parameter("event_topic", "")
        self.declare_parameter("loop_duration", 0.01)
        self.declare_parameter("heartbeat_duration", 0.5)
        self.declare_parameter("bhv_server_name", "bhv_server")

        use_sim_time = self.get_parameter("use_sim_time").value
        self.get_logger().info(f"use_sim_time: {use_sim_time}")

        self.event_topic = self.get_parameter("event_topic").value
        assert self.event_topic, f"{self.get_name()}: no 'event_topic' param specified"
        self.get_logger().info(f"Event topic: {self.event_topic}")

        self.loop_duration = self.get_parameter("loop_duration").value
        self.heartbeat_duration = self.get_parameter("heartbeat_duration").value
        assert (
            self.loop_duration > 0 and self.heartbeat_duration > 0
        ), f"Negative duration: hearbeat={self.heartbeat_duration}, loop={self.loop_duration}"
        assert self.loop_duration * 3 < self.heartbeat_duration, (
            f"Hearbeat duration ({self.heartbeat_duration}) must be at least"
            f" 3 times loop duration ({self.loop_duration})"
        )
        self.get_logger().info(
            f"Duration: hearbeat={self.heartbeat_duration}, loop={self.loop_duration}"
        )

        self.server_name = self.get_parameter("bhv_server_name").value
        self.get_logger().info(f"Behaviour server name: {self.server_name}")

        self.evt_pub = self.create_publisher(
            msg_type=Event, topic=self.event_topic, qos_profile=10
        )

        self._action_server = ActionServer(
            self, Behaviour, self.server_name, self.exec_cb
        )

    def exec_cb(self, goal_handle):
        response = Behaviour.Result()
        feedback = Behaviour.Feedback()

        pp_fsm = create_fsm()
        ud = UserData()

        start = time.time()
        loop_timeout = start + self.loop_duration
        heartbeat_timeout = start + self.heartbeat_duration
        while True:
            if pp_fsm.current_state_index == StateID.S_EXIT:
                goal_handle.succeed()
                response.result = ud.succeeded
                break

            if goal_handle.is_cancel_requested:
                produce_event(
                    event_data=pp_fsm.event_data, event_index=EventID.E_PREEMPTED
                )

            # execute behaviour
            fsm_mockup_bhv(fsm=pp_fsm, ud=ud)

            # State transitions
            reconfig_event_buffers(pp_fsm.event_data)
            fsm_step(pp_fsm)

            # Ensure loop rate & produce step event
            now = time.time()
            while now < loop_timeout:
                now = time.time()
            while loop_timeout < now:
                loop_timeout += self.loop_duration
            produce_event(pp_fsm.event_data, EventID.E_STEP)

            # Heartbeat timer for callback message
            if heartbeat_timeout < now:
                while heartbeat_timeout < now:
                    heartbeat_timeout += self.heartbeat_duration
                feedback.status = (
                    f"current state: {StateID(pp_fsm.current_state_index).name}"
                )
                goal_handle.publish_feedback(feedback)

        return response


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
