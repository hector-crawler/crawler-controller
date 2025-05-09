#!/usr/bin/env python3

from flask import Flask, send_file, request
from flask_cors import CORS
from flask_sock import Sock
import json
import os
import logging
from threading import Thread

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Bool, Int32


app = Flask(__name__)
CORS(app)
sock = Sock(app)


logger = logging.getLogger("werkzeug")
logger.setLevel(logging.WARN)

publisher = None
subscriber = None


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

@app.route("/api/rl/start", methods=["POST"])
def api_rl_start():
    #todo: implement
    # starts a rl node (probably needs some configuration from request body)
    return "ok"

@app.route("/api/rl/stop", methods=["POST"])
def api_rl_stop():
    #todo: implement
    # stops the rl node
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


ws_blinker_state = WebSocketStateBroadcast(
    {
        "blinker": False,
        "armPosition": 50,
        "handPosition": 50,
        "leftEncoderPosition": 0,
        "rightEncoderPosition": 0,
    }
)


@sock.route("/api/manual/state")
def api_blinker_state(ws):
    ws_blinker_state.handle_connection(ws)


# ROS publisher


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

    def blinker_state(self, msg):
        ws_blinker_state.update_state({"blinker": msg.data})

    def arm_position(self, msg):
        ws_blinker_state.update_state({"armPosition": msg.data})

    def hand_position(self, msg):
        ws_blinker_state.update_state({"handPosition": msg.data})

    def left_encoder_position(self, msg):
        ws_blinker_state.update_state({"leftEncoderPosition": msg.data})

    def right_encoder_position(self, msg):
        ws_blinker_state.update_state({"rightEncoderPosition": msg.data})
