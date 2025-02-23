#!/usr/bin/env python3
#-*- coding: utf-8 -*-

from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import sh1106
from PIL import ImageFont
import torch
import random
import rospy
from std_msgs.msg import Int32
from dynamixel_sdk import *
import time
import os
import subprocess
import csv
from datetime import datetime
from std_msgs.msg import String

#Log File name and Headers
log_filename = '/home/mars/catkin_ws/src/crawler_controller/scripts/logs/logfile.csv'    
headers = ['Timestamp','Episode', 'Progress']


# OLED-Konstanten
serial = i2c(port=1, address=0x3C)
device = sh1106(serial)
oled_font = ImageFont.truetype('FreeSans.ttf', 14)

# Sensoren einbinden
left_encoder_value = 0

# Dynamixel-Motor-Konstanten
PROTOCOL_VERSION = 2.0
DXL_ID_1, DXL_ID_3 = 1, 3
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0'
TORQUE_ENABLE, TORQUE_DISABLE = 1, 0

# DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE = 0, 2000

ADDR_TORQUE_ENABLE, ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION = 64, 116, 132

MOTOR_LIMITS = {
    DXL_ID_1: {'min': 600, 'max': 1200},
    DXL_ID_3: {'min': 1500, 'max': 2100}
}


portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def enable_all_motors():
    for motor_id in [DXL_ID_1, DXL_ID_3]:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to enable torque for Motor {motor_id}")
            error_pub.publish("Not able to properly enable motors")
        elif dxl_error != 0:
            print(f"Error occurred while enabling torque for Motor {motor_id}: {packetHandler.getRxPacketError(dxl_error)}")
            error_pub.publish("Not able to properly enable motors")
        else:
            print(f"Motor {motor_id} is now stiff (torque enabled).")

def initialize_motors():
    if portHandler.openPort():
        print("Port opened")
    else:
        raise Exception("Failed to open the port")

    if portHandler.setBaudRate(BAUDRATE):
        print("Baudrate set to", BAUDRATE)
    else:
        raise Exception("Failed to set baudrate")

    enable_all_motors()

last_encoder_position = None  # Initialer Wert; wird beim ersten Durchlauf gesetzt

def get_reward_from_encoder_position(current_enc_position):
    global last_encoder_position
    
    # Beim ersten Durchlauf wird nur der aktuelle Wert gespeichert
    if last_encoder_position is None:
        last_encoder_position = current_enc_position
        return 0  # Keine Belohnung beim ersten Durchlauf

    # Differenz berechnen
    difference = current_enc_position - last_encoder_position
    # Letzten Wert für den nächsten Durchlauf aktualisieren
    last_encoder_position = current_enc_position

    # Belohnung zuweisen
    if difference > 0:
        return 5                     # Wert für Belohnung bei Vorwärtsbewegung
    elif difference < 0:
        return -1
    else:
        return 0


# Q-Learning Setup
num_states, num_actions = 9, 4
q_table = torch.zeros([num_states, num_actions], dtype=torch.float32)
learning_rate = 0.5                # learning Rate verändern, damit Agent schneller auf neue Informationen reagiert
discount_factor = 0.99             # Diskontinierungsfaktor verändern, um zukünftige Belohnungen stärker zu gewichten
exploration_rate = 1.0    
max_exploration_rate, min_exploration_rate, exploration_decay_rate = 0.5, 0.01, 0.01

def left_encoder_callback(data):
    global left_encoder_value
    left_encoder_value = data.data

def discretize_motor_position(motor_pos, motor_min):
    return int((motor_pos - motor_min) / 250)

def state_from_motor_positions():
    motor1_pos = discretize_motor_position(read_motor_position(DXL_ID_1), MOTOR_LIMITS[DXL_ID_1]['min'])
    motor2_pos = discretize_motor_position(read_motor_position(DXL_ID_3), MOTOR_LIMITS[DXL_ID_3]['min'])
    state = motor1_pos * 5 + motor2_pos
    return min(state, num_states - 1)


def choose_action(state):
    if random.uniform(0, 1) < exploration_rate:
        return random.choice(range(num_actions))  # Erkunden
    return torch.argmax(q_table[state]).item()  # Ausnutzen

def learn(state, action, reward, next_state):
    predicted_value = q_table[state, action]
    target_value = reward + discount_factor * torch.max(q_table[next_state])
    q_table[state, action] = predicted_value + learning_rate * (target_value - predicted_value)

def execute_action_on_motors_and_get_feedback(action, state):
    if action == 0:
        adjust_motor_position(DXL_ID_1, 250)
    elif action == 1:
        adjust_motor_position(DXL_ID_1, -250)
    elif action == 2:
        adjust_motor_position(DXL_ID_3, 250)
    elif action == 3:
        adjust_motor_position(DXL_ID_3, -250)

    time.sleep(0.5) # Pause nach dem Bewegen der Motoren

    #print("Feedback")
    feedback = read_sensor_value()  # Implement 'read_sensor_value()' Funktion
    #print("Reward")
    reward = get_reward_from_encoder_position(feedback)  # Belohnung berechnen
    #print("Next state")
    next_state = state_from_motor_positions()
    #print("done test")
    #print(next_state, state)
    done = next_state == state  # Beispielhaftes Kriterium
    return next_state, reward, done

def update_oled_display(message):
    with canvas(device) as draw:
        draw.rectangle(device.bounding_box, outline="white", fill="black")
        draw.text((10, 10), message, font=oled_font, fill="white")

def read_sensor_value():
    return left_encoder_value

def read_motor_position(id, retries=2):
    for _ in range(retries):
        dxl_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION)
        if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
            return dxl_position
        time.sleep(0.5)  # Kurze Pause zwischen Wiederholungsversuchen
    print("Dynamixel communication error:", packetHandler.getTxRxResult(dxl_comm_result))
    return None



def adjust_motor_position(id, step):
    current_position = read_motor_position(id)
    desired_position = current_position + step
    
    min_limit = MOTOR_LIMITS[id]['min']
    max_limit = MOTOR_LIMITS[id]['max']
    
    desired_position = max(min_limit, min(desired_position, max_limit))
    
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, desired_position)
    return desired_position


def start_countdown(count_param):
    for i in range(count_param, 0, -1):
        update_oled_display(str(i))
        time.sleep(1)
    update_oled_display("Running 3x3")

def shutdown_countdown():
    update_oled_display("Shutting down...")
    time.sleep(2)  # Warten Sie eine Sekunde, um die Nachricht anzuzeigen.
    for i in range(3, 0, -1):
        update_oled_display(str(i))
        time.sleep(1)


def motors_reachable():
    try:
        for motor_id in [DXL_ID_1, DXL_ID_3]:
            position = read_motor_position(motor_id)
            if position is None:
                return False
        return True
    except:
        return False

def shutdown_raspberry():
    rospy.signal_shutdown("Shutting down due to motors not reachable.")
    time.sleep(5)
    os.system("sudo shutdown -h now")



def shutdown_with_password(password):
    command = 'sudo -S shutdown -h now'
    sudo_password = password

    process = subprocess.Popen(command.split(), stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    process.communicate(sudo_password.encode())

shutdown_password = "a205a205"

def turn_off_oled_display():
    with canvas(device) as draw:
         draw.rectangle(device.bounding_box, outline="black", fill="black")

def safe_shutdown():
    print("Motors not reachable. Shutting down.")
    shutdown_countdown()  # Countdown anzeigen
    shutdown_procedure()  # Motoren ausschalten
    turn_off_oled_display() # OLED Display ausschalten
    # shutdown_raspberry()  # Raspberry Pi herunterfahren
    shutdown_with_password(shutdown_password)

def shutdown_procedure():
    # Hier wird das Drehmoment für alle Motoren beim Herunterfahren deaktiviert
    for motor_id in [DXL_ID_1, DXL_ID_3]:
        packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    print("Motors are turned off. Goodbye!")
    
# Function to add a log entry
def add_log(episode,progress):
    # Create a new log entry with the current timestamp
    log_entry = (datetime.now(),episode, progress)
    log_string = (str(datetime.now()) + "," + str(episode) + "," + str(progress))
    pub.publish(log_string)
    
    # Open the file in append mode and write the log entry
    with open(log_filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(log_entry)

def main():
    rospy.init_node("move_control")
    count_param = rospy.get_param('~param1', 'default_value1')
    print("parameter")
    print(count_param)
    rospy.Subscriber("left_encoder_value", Int32, left_encoder_callback)
    
    global pub
    pub = rospy.Publisher('graph_data', String, queue_size=10)
    
    global error_pub
    error_pub = rospy.Publisher("motor_errors", String, queue_size=10)
    
    
    rospy.on_shutdown(shutdown_procedure)

    try:
        initialize_motors()
    except Exception as e:
        print("Error initializing motors:", e)
        return
        
    with open(log_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(headers)    
    
    start_countdown(count_param)
    update_oled_display("Running 3x3")
    total_episodes = 1000
    #starting time stamp
    add_log(0,0)
    #print(logs)
    global episode
    
    for episode in range(total_episodes):
        state = state_from_motor_positions()
        done = False
        
        #Speicherung des Reifenstandes vor der Episode
        first_pos_in_episode = left_encoder_value
        
	
        while not done:
            action = choose_action(state)
            #log action
            try:
                print(action, state)
                next_state, reward, done = execute_action_on_motors_and_get_feedback(action, state)
            except Exception as e:
                print("Exception: ") 
                print(next_state, reward, done, action, state)
                print(str(e))
                time.sleep(10)
                safe_shutdown()
                exit()
                break
            learn(state, action, reward, next_state)
            state = next_state
            print(read_motor_position(DXL_ID_1))

            # Überprüfen, ob die Motoren noch erreichbar sind
            if not motors_reachable():
                safe_shutdown()
                exit()
                break
        
        #Speicherung des Reifenstandes nach der Episode
        last_pos_in_episode = left_encoder_value
        episode_difference = last_pos_in_episode - first_pos_in_episode
        add_log(episode, episode_difference)
        
        exploration_rate = min_exploration_rate + (max_exploration_rate - min_exploration_rate) * torch.exp(torch.tensor(-exploration_decay_rate * episode))

    
   # rospy.spin()
    """
    while True:
        print(read_motor_position(DXL_ID_2))
        if not motors_reachable():
            exit()
            break
    """

if __name__ == '__main__':
    main()
