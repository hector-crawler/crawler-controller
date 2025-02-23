#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Int32

# Pins definieren
A_PIN = 16  # Beispiel-Pin für A
B_PIN = 20  # Beispiel-Pin für B

encoder_position = 0
last_channel_a = GPIO.LOW

def encoder_callback(channel):
    global encoder_position, last_channel_a

    # Aktueller Zustand von Kanal A und B auslesen
    channel_a = GPIO.input(A_PIN)
    channel_b = GPIO.input(B_PIN)

    # Wenn sich Kanal A verändert hat und nicht dem letzten Zustand entspricht
    if channel == A_PIN and channel_a != last_channel_a:
        if channel_a == channel_b:
            encoder_position -= 1                       # += Belohnung vorwärts -= Belohnung rückwärts 
        else:
            encoder_position += 1                       #  -= Belohnung vorwärts += Belohnung rückwärts 

    last_channel_a = channel_a

def sensor_node():
    rospy.init_node('left_encoder')  # Knotenname für linken Sensor; ändern Sie für rechten Sensor
    pub = rospy.Publisher('left_encoder_value', Int32, queue_size=10)  # Thema für linken Sensor

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Interrupts zum Erfassen von Flankenänderungen hinzufügen mit Entprellung (bouncetime)
    GPIO.add_event_detect(A_PIN, GPIO.BOTH, callback=encoder_callback, bouncetime=10)
    GPIO.add_event_detect(B_PIN, GPIO.BOTH, callback=encoder_callback, bouncetime=10)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        #print("encoder position")
        #print(encoder_position)
        pub.publish(encoder_position)
        rate.sleep()

    GPIO.cleanup()

if __name__ == '__main__':
    try:
        sensor_node()
    except rospy.ROSInterruptException:
        pass

