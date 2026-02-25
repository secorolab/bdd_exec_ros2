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

from bdd_ros2_interfaces.msg import TrinaryStamped
from rosidl_runtime_py.utilities import get_message
from rdflib import Graph, Literal, URIRef
from rdf_utils.models.common import ModelBase
from bdd_dsl.models.clauses import FluentClauseModel
from bdd_exec_ros2.urirefs import (
    URI_ROS_PRED_MSG_TYPE,
    URI_ROS_PRED_TOPIC_NAME,
    URI_ROS_TYPE_TOPIC,
)


def load_ros_topic_model(model: ModelBase, graph: Graph):
    topic_name = graph.value(
        subject=model.id, predicate=URI_ROS_PRED_TOPIC_NAME, any=False
    )
    assert isinstance(
        topic_name, Literal
    ), f"'topic_name' of '{model.id}' not a Literal: {topic_name}"
    model.set_attr(key=URI_ROS_PRED_TOPIC_NAME, val=topic_name.toPython())

    msg_type_str = graph.value(
        subject=model.id, predicate=URI_ROS_PRED_MSG_TYPE, any=False
    )
    assert isinstance(
        msg_type_str, Literal
    ), f"'message-type' of '{model.id}' not a Literal: {msg_type_str}"

    msg_type = get_message(msg_type_str)
    model.set_attr(key=URI_ROS_PRED_MSG_TYPE, val=msg_type)


class ObservationManager(object):
    trinary_bus: dict[URIRef, list[TrinaryStamped]]

    def __init__(self) -> None:
        self.trinary_bus = {}

    def load_fluent_obs(self, fc: FluentClauseModel, graph: Graph):
        assert (
            URI_ROS_TYPE_TOPIC in fc.types
        ), "currently only support observation policy from trinary ROS topics"

        if fc.id not in self.trinary_bus:
            self.trinary_bus[fc.id] = []

        load_ros_topic_model(model=fc, graph=graph)

    def update_fpolicy_assertion(self, fc_uri: URIRef, trinary: TrinaryStamped):
        pass
