#!/usr/bin/env python3

import json
import logging
import os
from threading import Thread

import rclpy
import rclpy.logging
from crawler_msgs.msg import (  # type: ignore
    QLearningInternalState,
    QLearningParameters,
    RLEnvironmentInternals,
)
from flask import Flask, request, send_file
from flask_cors import CORS
from flask_sock import Sock  # type: ignore
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Int32  # type: ignore

# ROS nodes


class WebApiPublisher(Node):
    def __init__(self):
        super().__init__("crawler_web_api_publisher")

        self.blinker_toggle_publisher = self.create_publisher(
            Empty, "/crawler/blinker/toggle", 5
        )
        self.blinker_write_publisher = self.create_publisher(
            Bool, "/crawler/blinker/write", 5
        )
        self.arm_move_publisher = self.create_publisher(Int32, "/crawler/arm/move", 5)
        self.hand_move_publisher = self.create_publisher(Int32, "/crawler/hand/move", 5)
        self.left_encoder_mock_publisher = self.create_publisher(
            Int32, "/crawler/left_encoder/mock", 5
        )
        self.right_encoder_mock_publisher = self.create_publisher(
            Int32, "/crawler/right_encoder/mock", 5
        )
        self.q_learning_start_publisher = self.create_publisher(
            QLearningParameters, "/crawler/rl/q_learning/start", 5
        )
        self.rl_stop_publisher = self.create_publisher(Empty, "/crawler/rl/stop", 5)

    def blinker_toggle(self):
        self.blinker_toggle_publisher.publish(Empty())

    def blinker_write(self, state):
        self.blinker_write_publisher.publish(Bool(data=state))

    def arm_move(self, step):
        self.arm_move_publisher.publish(Int32(data=step))

    def hand_move(self, step):
        self.hand_move_publisher.publish(Int32(data=step))

    def left_encoder_mock(self, position):
        self.left_encoder_mock_publisher.publish(Int32(data=position))

    def right_encoder_mock(self, position):
        self.right_encoder_mock_publisher.publish(Int32(data=position))

    def start_rl_q_learning(
        self,
        hand_states: int,
        arm_states: int,
        hand_step: int,
        arm_step: int,
        learning_rate: float,
        explor_rate: float,
        explor_decay_factor: float,
        min_explor_rate: float,
        discount_factor: float,
    ):
        self.q_learning_start_publisher.publish(
            QLearningParameters(
                hand_states=hand_states,
                arm_states=arm_states,
                hand_step=hand_step,
                arm_step=arm_step,
                learning_rate=learning_rate,
                explor_rate=explor_rate,
                explor_decay_factor=explor_decay_factor,
                min_explor_rate=min_explor_rate,
                discount_factor=discount_factor,
            )
        )

    def stop_rl(self):
        self.rl_stop_publisher.publish(Empty())
        ws_rl_internals.update_state({"qLearning": None})
        self.get_logger().info("Stopping RL")


class WebApiSubscriber(Node):
    def __init__(self):
        super().__init__("crawler_web_api_subscriber")

        self.create_subscription(Bool, "/crawler/blinker/state", self.blinker_state, 5)
        self.create_subscription(Int32, "/crawler/arm/position", self.arm_position, 5)
        self.create_subscription(Int32, "/crawler/hand/position", self.hand_position, 5)
        self.create_subscription(
            Int32, "/crawler/left_encoder/position", self.left_encoder_position, 5
        )
        self.create_subscription(
            Int32, "/crawler/right_encoder/position", self.right_encoder_position, 5
        )
        self.create_subscription(
            RLEnvironmentInternals, "/crawler/rl/internals", self.rl_internals, 5
        )
        self.create_subscription(
            QLearningInternalState,
            "/crawler/rl/q_learning/internals",
            self.rl_q_learning_internals,
            5,
        )

    def blinker_state(self, msg):
        ws_manual_state.update_state({"blinker": msg.data})

    def arm_position(self, msg):
        ws_manual_state.update_state({"armPosition": msg.data})

    def hand_position(self, msg):
        ws_manual_state.update_state({"handPosition": msg.data})

    def left_encoder_position(self, msg):
        ws_manual_state.update_state({"leftEncoderPosition": msg.data})

    def right_encoder_position(self, msg):
        ws_manual_state.update_state({"rightEncoderPosition": msg.data})

    def rl_internals(self, msg):
        ws_rl_internals.update_state(
            {
                "rlEnvironmentInternals": {
                    "loopState": msg.loop_state,
                    "latestStateReward": {
                        "armPosition": msg.latest_state_reward.arm_position,
                        "handPosition": msg.latest_state_reward.hand_position,
                        "reward": msg.latest_state_reward.reward,
                    },
                    "latestAction": {
                        "moveArm": msg.latest_action.move_arm,
                        "moveHand": msg.latest_action.move_hand,
                    },
                    "progress": msg.progress.tolist(),
                }
            }
        )

    def rl_q_learning_internals(self, msg):
        ws_rl_internals.update_state(
            {
                "qLearning": {
                    "armStates": msg.arm_states,
                    "handStates": msg.hand_states,
                    "armStep": msg.arm_step,
                    "handStep": msg.hand_step,
                    "learningRate": msg.learning_rate,
                    "explorationRate": msg.explor_rate,
                    "explorationDecayFactor": msg.explor_decay_factor,
                    "minExplorationRate": msg.min_explor_rate,
                    "discountFactor": msg.discount_factor,
                    "qTableRows": msg.q_table_rows,
                    "qTableCols": msg.q_table_cols,
                    "qTableValues": msg.q_table_values.tolist(),
                    "moveIsExploration": msg.move_is_exploration,
                }
            }
        )


# web server

app = Flask(__name__)
CORS(app)
sock = Sock(app)


logger = logging.getLogger("werkzeug")
logger.setLevel(logging.WARN)

publisher: WebApiPublisher = None  # type: ignore
subscriber: WebApiSubscriber = None  # type: ignore


def main(args=None):
    rclpy.init(args=args)
    global publisher
    publisher = WebApiPublisher()
    rclpy.spin_once(publisher, timeout_sec=0)

    Thread(target=spin_subscriber).start()

    app.run(host="0.0.0.0")


def spin_subscriber():
    global subscriber
    subscriber = WebApiSubscriber()
    rclpy.spin(subscriber)


# serve single page web app


@app.route("/", defaults={"path": ""})
@app.route("/<path:path>")
def serve_single_page_app(path):
    if "assets" not in path:
        path = "index.html"
    return send_file(os.path.join("web-ui", path))


@app.route("/favicon.ico")
def favicon():
    return send_file(os.path.join("web-ui", "favicon.ico"))


@app.route("/buildMetadata")
def get_build_metadata():
    return os.environ.get("CRAWLER_BUILD_METADATA", "unknown build")


# API routes


@app.route("/api/manual/blinker/toggle", methods=["POST"])
def api_blink():
    publisher.blinker_toggle()
    return "ok"


@app.route("/api/manual/blinker/write", methods=["POST"])
def api_blinker_write():
    state = request.json.get("state")
    publisher.blinker_write(state)
    return "ok"


@app.route("/api/manual/moveArm", methods=["POST"])
def api_move_arm():
    step = request.json.get("step")
    publisher.arm_move(step)
    return "ok"


@app.route("/api/manual/moveHand", methods=["POST"])
def api_move_hand():
    step = request.json.get("step")
    publisher.hand_move(step)
    return "ok"


@app.route("/api/manual/mockLeftEncoder", methods=["POST"])
def api_mock_left_encoder():
    position = request.json.get("position")
    publisher.left_encoder_mock(position)
    return "ok"


@app.route("/api/manual/mockRightEncoder", methods=["POST"])
def api_mock_right_encoder():
    position = request.json.get("position")
    publisher.right_encoder_mock(position)
    return "ok"


@app.route("/api/rl/start/qLearning", methods=["POST"])
def api_rl_start():
    arm_states = request.json.get("armStates")
    hand_states = request.json.get("handStates")
    arm_step = request.json.get("armStep")
    hand_step = request.json.get("handStep")
    learning_rate = float(request.json.get("learningRate"))
    explor_rate = float(request.json.get("explorationRate"))
    explor_decay_factor = float(request.json.get("explorationDecayFactor"))
    min_explor_rate = float(request.json.get("minExplorationRate"))
    discount_factor = float(request.json.get("discountFactor"))
    publisher.start_rl_q_learning(
        arm_states,
        hand_states,
        hand_step,
        arm_step,
        learning_rate,
        explor_rate,
        explor_decay_factor,
        min_explor_rate,
        discount_factor,
    )
    return "ok"


@app.route("/api/rl/stop", methods=["POST"])
def api_rl_stop():
    publisher.stop_rl()
    return "ok"


class WebSocketStateBroadcast:
    def __init__(self, initial_state):
        self.connections = []
        self.state = initial_state

    def handle_connection(self, ws):
        self.connections.append(ws)
        ws.send(json.dumps(self.state))
        try:
            while True:
                ws.receive()
        finally:
            self.connections.remove(ws)

    def update_state(self, partial_update):
        self.state.update(partial_update)
        for ws in self.connections:
            ws.send(json.dumps(self.state))


ws_manual_state = WebSocketStateBroadcast(
    {
        "blinker": False,
        "armPosition": 50,
        "handPosition": 50,
        "leftEncoderPosition": 0,
        "rightEncoderPosition": 0,
    }
)

ws_rl_internals = WebSocketStateBroadcast(
    {
        "rlEnvironmentInternals": None,
        "qLearning": None,
    }
)


@sock.route("/api/manual/state")
def api_blinker_state(ws):
    ws_manual_state.handle_connection(ws)


@sock.route("/api/rl/internals")
def api_rl_internals(ws):
    ws_rl_internals.handle_connection(ws)
