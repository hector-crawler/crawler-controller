#!/usr/bin/env python3

from flask import Flask, send_file
from flask_cors import CORS
import os
import logging

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


app = Flask(__name__)
CORS(app)

logger = logging.getLogger("werkzeug")
logger.setLevel(logging.WARN)

publisher = None

def main(args=None):
    print("Starting crawler_web_api")

    rclpy.init(args=args)
    global publisher
    publisher = WebApiPublisher()
    rclpy.spin_once(publisher, timeout_sec=0)
    
    app.run(host="0.0.0.0")


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

@app.route("/api/blink", methods=["POST"])
def api_blink():
    publisher.blink()
    return "blinked"

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


# ROS publisher

class WebApiPublisher(Node):
    
    def __init__(self):
        super().__init__("crawler_web_api_publisher")
        self.blink_publisher = self.create_publisher(Empty, "crawler_blinker_toggle", 5)

    def blink(self):
        self.blink_publisher.publish(Empty())
        self.get_logger().info("Publishing blink")