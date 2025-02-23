#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import sh1106
from PIL import ImageFont
from std_msgs.msg import String

# OLED-Konstanten
serial = i2c(port=1, address=0x3C)
device = sh1106(serial)
oled_font = ImageFont.truetype('FreeSans.ttf', 14)

def update_oled_display(message):
    with canvas(device) as draw:
        draw.rectangle(device.bounding_box, outline="white", fill="black")
        draw.text((10, 10), message, font=oled_font, fill="white")
 
def monitor_callback(data):
	#print("in callback monitor")
	message = data.data
	update_oled_display(message)
	
        
def main():
    rospy.init_node("Monitor_node")
    rospy.Subscriber("monitor_message", String, monitor_callback)
    rospy.spin()
	

if __name__ == '__main__':
    main()
