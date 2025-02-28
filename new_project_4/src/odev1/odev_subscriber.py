#!/usr/bin/env python3
import rospy
from ros_tutorials.msg import RobotSensor
import math

# Hedef koordinatları
target_x = 10.0
target_y = 10.0

def callback(data):
    global velocity
    
    # Hedefe kalan mesafeyi hesapla
    distance_to_target = math.sqrt((target_x - data.x)**2 + (target_y - data.y)**2)
    
    rospy.loginfo(f"Received velocity: {data.velocity}, position: ({data.x}, {data.y})")
    rospy.loginfo(f"Distance to target: {distance_to_target:.2f} meters")
    
    # Eğer robot hedefe ulaştıysa hem subscriber'ı hem de publisher'ı durdur
    if distance_to_target < 0.1:
        rospy.loginfo("Robot has reached the target! Stopping...")
    else:
        rospy.loginfo(f"Robot is {distance_to_target:.2f} meters away from the target")

def robot_subscriber():  
    rospy.init_node('robot_subscriber', anonymous=True)# Node başlatılıyor
    rospy.Subscriber('robot_sensor_topic', RobotSensor, callback) # Subscriber oluşturuluyor  
    rospy.spin() # ROS'un sonsuz döngüsünü başlat

if __name__ == '__main__':
    robot_subscriber()
