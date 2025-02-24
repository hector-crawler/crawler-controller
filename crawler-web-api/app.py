#!/usr/bin/env python3

from flask import Flask, send_file, request
from flask_cors import CORS
import os
import logging

app = Flask(__name__)
CORS(app)

logger = logging.getLogger("werkzeug")
logger.setLevel(logging.WARN)


# serve single page app

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

@app.route("/api/start", methods = ["POST"])
def api_start():
    return "started"

@app.route("/api/stop", methods = ["POST"])
def api_stop():
    return "stopped"

@app.route("/api/moveArm", methods = ["POST"])
def api_move_arm():
    return "arm moved"

@app.route("/api/moveHand", methods = ["POST"])
def api_move_hand():
    return "hand moved"