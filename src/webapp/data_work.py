#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import matplotlib
matplotlib.use('Agg')  # Use a non-interactive backend
import matplotlib.pyplot as plt



# Global list to store the last 20 episodes of ride data
episodes = []
progresses = []

def callback(data):
    global episodes
    global progresses

    # Extract relevant data
    ride_data = data.data
    # Add the new data point to the list
    timestamp_str, episode_str, progress_str = ride_data.split(',')
    episodes.append(int(episode_str))
    progresses.append(int(progress_str))

    # Keep only the last 20 episodes
    #if len(episodes) > 20:
        #episodes.pop(0)
        #progresses.pop(0)
        
    # Create the plot
    plt.figure(figsize=(10, 6))
    plt.plot(episodes, progresses, marker='o', linestyle='-', color='b', label='Progress')
    # Add titles and labels
    plt.title('Progress Over Episodes')
    plt.xlabel('Episode')
    plt.ylabel('Progress')
    plt.grid(True)
    plt.legend()
    
    # Save the plot as a PNG file
    plt.savefig('/home/mars/catkin_ws/src/crawler_controller/scripts/webapp/static/images/progress_over_episodes.png')
    #print("saved that file <3")

def main():
    rospy.init_node('graphing_node')
    
    rospy.Subscriber('graph_data', String, callback)
    
    rospy.spin()
	

if __name__ == "__main__":
    main()
