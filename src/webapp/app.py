#!/usr/bin/env python3
#-*- coding: utf-8 -*-

from flask import Flask, render_template,jsonify, request, url_for, flash, redirect
from werkzeug.exceptions import abort
import subprocess
import os
import signal
import rospy
from std_msgs.msg import String, Int32
from flask_sock import Sock
import socket


print("default application")

app = Flask(__name__)
app.config['SECRET_KEY'] = 'Kei'
sock = Sock(app)
recent_data = ""
ws_connection = None
ws_connection2 = None
ws_connection_arm = None
ws_connection_hand = None

# Global robot state
robot_state = {"running": False, "data": "No data yet"}


def get_local_ip():
    # Erstellt einen Dummy-Socket, um die IP-Adresse zu ermitteln
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Verbindet den Socket mit einer öffentlichen Adresse (8.8.8.8:80 ist ein Google DNS)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip


def start_robot():
    global robot_state
    global pid
    global pid_encoder
    global pid_graph
    #print("parameter:")
    print("in the starting realm")
    #print(cnt)
    #cnt = request.form['countdown']
    params = [
        "_param1:={}".format(cnt),
        "_param2:={}".format(lrate),
        "_param3:={}".format(dfactor)
    ]
    print(params)
    pid_encoder = start_ros_node('crawler_controller', 'encoder_left.py', 'Sensoren_node', params)
    pid = start_ros_node('crawler_controller', 'q_learning_3x3.py', 'Q_Learning', params)
    #pid = start_ros_node('crawler_controller', 'manuel_control.py', 'Manuel', params)
    pid_graph = start_ros_node('crawler_controller', 'data_work.py', 'Graph_node', params)
    print("after starting the node")
    robot_state["running"] = True
    robot_state["data"] = "Robot started"
    
def start_robot_manuel():
    global robot_state
    global pid
    global pid_encoder
    
    params = [                     #parameters aren't necessary for the node but otherwise the start fucntion doesn't work
        "_param1:={}".format(cnt),
        "_param2:={}".format(lrate),
        "_param3:={}".format(dfactor)
    ]

    pid_encoder = start_ros_node('crawler_controller', 'encoder_left.py', 'Sensoren_node', params)
    pid = start_ros_node('crawler_controller', 'manuel_control.py', 'Manuel', params)
    print("after starting the manuel node")
    robot_state["running"] = True
    robot_state["data"] = "Robot started"
    
def stop_robot_manuel():
    global robot_state
    stop_ros_node(pid, 'manuel_control.py')
    stop_ros_node(pid_encoder, 'encoder_left.py')
    robot_state["running"] = False
    robot_state["data"] = "Robot stopped"
    pubMonitor.publish(str(local_ip)+"\n:5000")

def stop_robot():
    global robot_state
    stop_ros_node(pid, 'q_learning_3x3.py')
    #stop_ros_node(pid, 'manuel_control.py')
    stop_ros_node(pid_encoder, 'encoder_left.py')
    stop_ros_node(pid_graph, 'data_work.py')
    robot_state["running"] = False
    robot_state["data"] = "Robot stopped"
    pubMonitor.publish(str(local_ip)+"\n:5000")

def get_robot_data():
    return robot_state["data"]
    
def start_ros_node(node_package, node_type, node_name, params=None):
    try:
        # Define the command to start the ROS node
        command = [
            'rosrun',
            node_package,
            node_type,
        ]
        
        # Add parameters if any
        if params:
            command.extend(params)

        # Set the environment for ROS
        env = os.environ.copy()
        env['ROS_PACKAGE_PATH'] = '/opt/ros/noetic/share:/opt/ros/noetic/stacks:/home/mars/catkin_ws/src'  # Change if needed
        env['ROS_MASTER_URI'] = 'http://localhost:11311'  # Change if needed
        env['ROS_PYTHON_LOG_CONFIG_FILE'] = '/opt/ros/noetic/etc/ros/python_logging.conf'  # Change if needed

        # Start the ROS node
        process = subprocess.Popen(command, env=env)

        print("Started ROS node '{}' with PID {}".format(node_name, process.pid))
        return process.pid  # Return the process ID

    except Exception as e:
        print("Failed to start ROS node '{}': {}".format(node_name, e))
        
# Function to stop the ROS node
def stop_ros_node(pid, node_name):
    try:
        os.kill(pid, signal.SIGKILL)  # Send the SIGTERM signal to the process
        print("Stopped ROS node '{}' with PID {}".format(node_name, pid))

    except Exception as e:
        print("Failed to stop ROS node '{}': {}".format(node_name, e))
        
def callback(data):
    global recent_data
    recent_data = data.data
    global ws_connection2
    if ws_connection2:
        try:
            # Send the data received from ROS topic to the WebSocket client
            ws_connection2.send(recent_data)
        except Exception as e:
            print(f"Failed to send data over WebSocket: {e}")
            ws_connection2 = None  # Reset the connection if sending fails

def error_callback(data):
    global robot_state
    global error_message
    error_message = data.data
    robot_state["running"] = False
    robot_state["data"] = str(error_message)
    print("in Callback")
    print(error_message)
    stop_ros_node(pid, 'q_learning_3x3.py')
    stop_ros_node(pid_encoder, 'encoder_left.py')
    stop_ros_node(pid_graph, 'data_work.py')
    print("stopped everything")
    global ws_connection
    if ws_connection:
        try:
            # Send the data received from ROS topic to the WebSocket client
            ws_connection.send(error_message)
            print("send the message")
        except Exception as e:
            print(f"Failed to send data over WebSocket: {e}")
            ws_connection = None  # Reset the connection if sending fails
            
def positionArm_callback(data):
    print("in the arm callback in the app")
    global positionArm
    positionArm = data.data
    global ws_connection_arm
    if ws_connection_arm:
        try:
            # Send the data received from ROS topic to the WebSocket client
            ws_connection_arm.send(positionArm)
            print("send the arm position")
        except Exception as e:
            print(f"Failed to send data over WebSocket: {e}")
            ws_connection_arm = None  # Reset the connection if sending fails
            
            
def positionHand_callback(data):
    print("in the Hand callback in the app")
    global positionHand
    positionHand = data.data
    global ws_connection_hand
    if ws_connection_hand:
        try:
            # Send the data received from ROS topic to the WebSocket client
            ws_connection_hand.send(positionHand)
            print("send the hand position")
        except Exception as e:
            print(f"Failed to send data over WebSocket: {e}")
            ws_connection_hand = None  # Reset the connection if sending fails


@app.route('/')
def index():
    local_ip = get_local_ip()
    print(f"Die IP-Adresse dieses Computers ist: {local_ip}")
    pubMonitor.publish(str(local_ip)+"\n:5000")
    return render_template('index.html')

@app.route('/automatik')
def automatik():
    print("send")
    return render_template('automatik.html')
    
@app.route('/manuel')
def manuel():
    return render_template('manuel.html')
    
@app.route('/moveArm', methods=['POST'])
def moveArm():
    step = request.form.get('step')
    print("step")
    print(step)
    pubArm.publish(int(step))

@app.route('/moveHand', methods=['POST'])
def moveHand():
    step = request.form.get('step') 
    print("hand")
    print("step")
    print(step)
    pubHand.publish(int(step))

@app.route('/graph')
def graph():
    return render_template('graph.html')


@app.route('/start', methods=['POST'])
def start():   
    global cnt
    global lrate
    global dfactor
    print("starting variables input") 
    print("Form data:", request.form)
    print("Request data:", request.data)  # Print raw data received

    cnt = request.form.get('countdown')
    lrate = request.form.get('lrate')
    dfactor = request.form.get('dfactor')
    print("lrate")
    print(lrate)
    print("cnt")
    print(cnt)
    print("dfactor")
    print(dfactor)
    if dfactor is None:
        return jsonify(status="error", message="ldiscount factor parameter missing"), 400
    if lrate is None:
        return jsonify(status="error", message="learning rate parameter missing"), 400
    if cnt is None:
        return jsonify(status="error", message="countdown parameter missing"), 400

    start_robot()
    return jsonify(status="started")

@app.route('/startManuel', methods=['POST'])
def startManuel():   
    start_robot_manuel()
    return jsonify(status="started")
    
@app.route('/stopManuel', methods=['POST'])
def stopManuel():
    stop_robot_manuel()
    return jsonify(status="stopped")


@app.route('/stop', methods=['POST'])
def stop():
    stop_robot()
    return jsonify(status="stopped")

@app.route('/data', methods=['GET'])
def data():
    data = get_robot_data()
    return jsonify(data=data)
    
@sock.route('/ws')
def ws(ws):
    global ws_connection
    ws_connection = ws
    print("WebSocket connection opened")
    # Keep the connection open to handle incoming messages (if any)
    while True:
        message = ws.receive()
        if message is None:
            break  # Connection closed, exit loop

    print("WebSocket connection closed")
    ws_connection = None
    
    
@sock.route('/ws2')
def ws2(ws2):
    global ws_connection2
    ws_connection2 = ws2
    print("WebSocket2 connection opened")
    # Keep the connection open to handle incoming messages (if any)
    while True:
        message = ws2.receive()
        if message is None:
            break  # Connection closed, exit loop

    print("WebSocket2 connection closed")
    ws_connection2 = None

@sock.route('/wsArm')
def wsArm(wsArm):
    global ws_connection_arm
    ws_connection_arm = wsArm
    print("WebSocketArm connection opened")
    # Keep the connection open to handle incoming messages (if any)
    while True:
        message = wsArm.receive()
        if message is None:
            break  # Connection closed, exit loop

    print("WebSocketArm connection closed")
    ws_connection_arm = None
    
@sock.route('/wsHand')
def wsHand(wsHand):
    global ws_connection_hand
    ws_connection_hand = wsHand
    print("WebSocketHand connection opened")
    # Keep the connection open to handle incoming messages (if any)
    while True:
        message = wsHand.receive()
        if message is None:
            break  # Connection closed, exit loop

    print("WebSocketArm connection closed")
    ws_connection_hand = None


if __name__ == '__main__':
    rospy.init_node('App_node')
    
    global pubArm
    pubArm = rospy.Publisher('motorArm', Int32, queue_size=10)
    
    global pubHand
    pubHand = rospy.Publisher('motorHand', Int32, queue_size=10)
    
    global pubMonitor
    pubMonitor = rospy.Publisher('monitor_message', String, queue_size=10)
    
    rospy.Subscriber('graph_data', String, callback)
    rospy.Subscriber("motor_errors", String, error_callback)
    rospy.Subscriber("positionArm", Int32, positionArm_callback)
    rospy.Subscriber("positionHand", Int32, positionHand_callback)
    from threading import Thread
    flask_thread = Thread(target=app.run, kwargs={'host': '0.0.0.0', 'port': 5000})
    flask_thread.start()
    
    global local_ip
    local_ip = get_local_ip()
    rospy.sleep(1) #necessary to have the monitor node up and running before the first callback
    
    pubMonitor.publish(str(local_ip)+"\n:5000")

    rospy.spin()
    #app.run(debug=True, host= '0.0.0.0')
