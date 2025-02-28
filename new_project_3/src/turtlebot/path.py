#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Path:
    def __init__(self):
        # ROS node'unu başlat
        rospy.init_node('yol', anonymous=True)
        
        # TurtleBot'un pozisyon bilgilerini dinlemek için subscriber
        self.pose_sub = rospy.Subscriber('/odom', Odometry, self.pose_callback)
        
        # Hareket edip etmediğini kontrol etmek için `cmd_vel` topiğinden veri al
        self.vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.vel_callback)

        # Başlangıç pozisyonları
        self.prev_x = None
        self.prev_y = None
        self.current_x = None
        self.current_y = None
        self.moving = False  # TurtleBot'un hareket edip etmediğini takip eden bir bayrak

        self.distance_threshold = 0.1  # Mesafe eşiği (10 cm)

    def pose_callback(self, msg):
        """Pozisyon bilgilerini al."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        if self.prev_x is None and self.prev_y is None:
            # İlk pozisyonu ayarla
            self.prev_x = self.current_x
            self.prev_y = self.current_y
            return

        # Anlık gidilen mesafeyi hesapla
        distance_x = self.current_x - self.prev_x
        distance_y = self.current_y - self.prev_y

        # Eşikleri aşarsa yazdır-
        if abs(distance_x) >= self.distance_threshold or abs(distance_y) >= self.distance_threshold:
            rospy.loginfo(f"Anlık Gidilen Mesafe - X: {distance_x:.2f}, Y: {distance_y:.2f}")

            # Pozisyonları güncelle
            self.prev_x = self.current_x
            self.prev_y = self.current_y

    def vel_callback(self, msg):
        """Hareket mesajlarını alarak hareket durumunu güncelle."""
        # Eğer hareket varsa, hareket bayrağını true yap
        if msg.linear.x != 0.0 or msg.angular.z != 0.0:
            self.moving = True
        else:
            self.moving = False

    def run(self):
        """Node'u başlat."""
        rospy.loginfo("Yol node'u çalışıyor...")
        rospy.spin()  # Sürekli olarak mesajları dinle

if __name__ == '__main__':
    try:
        path = Path()
        path.run()  # Mesafeyi güncellemeye başla
    except rospy.ROSInterruptException:
        pass

