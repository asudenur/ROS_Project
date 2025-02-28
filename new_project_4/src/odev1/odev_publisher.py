#!/usr/bin/env python3
import rospy
from ros_tutorials.msg import RobotSensor

# Hedef koordinatları
target_x = 10.0
target_y = 10.0

def robot_publisher():
    pub = rospy.Publisher('robot_sensor_topic', RobotSensor, queue_size=10)
    rospy.init_node('robot_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz, her saniyede bir mesaj gönderilecek

    # Başlangıç değerleri
    velocity = 1.0  # Sabit hız
    x = 0.0  # X ekseni başlangıç pozisyonu
    y = 0.0  # Y ekseni başlangıç pozisyonu

    # Robot hareket ediyor
    while not rospy.is_shutdown():
        # Pozisyon güncellemesi
        x += velocity  # Sabit hızda X ekseninde ilerleme
        y += velocity  # Sabit hızda Y ekseninde ilerleme

        # Mesaj oluştur ve yayınla
        sensor_msg = RobotSensor()
        sensor_msg.velocity = velocity
        sensor_msg.x = x
        sensor_msg.y = y

        rospy.loginfo(f"Publishing velocity: {velocity}, x: {x}, y: {y}")
        pub.publish(sensor_msg)
        
        if x==target_x and y==target_y:	
        	rospy.loginfo("Robot has reached the target! Stopping...")
        	break
        rate.sleep()

if __name__ == '__main__':
    try:
        robot_publisher()
    except rospy.ROSInterruptException:
        pass 
        
        

