#!/usr/bin/env python3

from flask import Flask, send_file
from flask_cors import CORS
from flask_sock import Sock
import json
import os
import logging
from threading import Thread

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Bool


app = Flask(__name__)
CORS(app)
sock = Sock(app)


logger = logging.getLogger("werkzeug")
logger.setLevel(logging.WARN)

publisher = None
subscriber = None

def main(args=None):
    print("Starting crawler_web_api")

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


# API routes

ws_blinker_state = []

@app.route("/api/blinker/toggle", methods=["POST"])
def api_blink():
    publisher.blinker_toggle()
    return "ok"

@app.route("/api/blinker/write", methods=["POST"])
def api_blinker_write(request):
    state = request.json.get("state")
    publisher.blinker_write(state)
    return "ok"

@app.route("/api/start", methods = ["POST"])
def api_start():
    #TODO implement
    return "started"

@app.route("/api/stop", methods = ["POST"])
def api_stop():
    #TODO implement
    return "stopped"

@app.route("/api/moveArm", methods = ["POST"])
def api_move_arm():
    #TODO implement
    return "arm moved"

@app.route("/api/moveHand", methods = ["POST"])
def api_move_hand():
    #TODO implement
    return "hand moved"

@sock.route("/state/blinker/state")
def api_blinker_state(ws):
    global ws_blinker_state
    ws_blinker_state.append(ws)
    try:
        while True:
            ws.receive()
    finally:
        ws_blinker_state.remove(ws)


# ROS publisher


class WebApiPublisher(Node):
    
    def __init__(self):
        super().__init__("crawler_web_api_publisher")

        self.blinker_toggle_publisher = self.create_publisher(Empty, "crawler_blinker_toggle", 5)
        self.blinker_write_publisher = self.create_publisher(Bool, "crawler_blinker_write", 5)

    def blinker_toggle(self):
        self.blinker_toggle_publisher.publish(Empty())

    def blinker_write(self, state):
        self.blinker_write_publisher.publish(Bool(data=state))


class WebApiSubscriber(Node):

    def __init__(self):
        super().__init__("crawler_web_api_subscriber")

        self.create_subscription(Bool, "crawler_blinker_state", self.blinker_state, 5)

    def blinker_state(self, msg):
        global ws_blinker_state
        for ws in ws_blinker_state:
            ws.send(json.dumps({ "state": msg.data }))
