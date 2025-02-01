#!/usr/bin/env python3

import rospy 
import os 
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist

class ObstacleAvoidance:
    def __init__(self):
        # ROS düğümünü başlat
        rospy.init_node('obstacle_avoidance', anonymous=True)
        
        # Robotun hareket komutlarını yayınlayacak bir publisher oluştur
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # LIDAR verilerini alacak bir subscriber oluştur
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Twist mesajı (hareket komutları) için bir nesne oluştur
        self.twist = Twist()
        
        # Robotun engellerden güvenli mesafesini tanımla (metre cinsinden)
        self.safe_distance = 0.5
        
        # Robotun döngü hızını ayarla (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Harita kaydetme durumu: Başlangıçta harita kaydedilmedi
        self.map_saved = False
        
        # Başlangıç zamanını kaydet
        self.start_time = rospy.Time.now()

    def scan_callback(self, scan_data):
        """
        LIDAR verilerini işler ve engel algılama için gerekli mesafeleri belirler.
        """
        # LIDAR tarafından algılanan mesafeleri al
        ranges = scan_data.ranges
        
        # Ön, sol ve sağ tarafın mesafelerini hesapla
        front = min(min(ranges[:30]), min(ranges[-30:]))  # Robotun ön tarafı
        left = min(ranges[60:120])  # Sol taraf
        right = min(ranges[240:300])  # Sağ taraf

        # Eğer ön tarafta engel varsa
        if front < self.safe_distance:
            self.twist.linear.x = 0.0  # Robotu durdur
            # Sol taraf daha açıksa sola dön, aksi halde sağa dön
            self.twist.angular.z = 0.5 if left > right else -0.5
        else:
            self.twist.linear.x = 0.2  # Engel yoksa ileri hareket et
            self.twist.angular.z = 0.0  # Dönüş yapma

        # Hareket komutlarını yayınla
        self.cmd_pub.publish(self.twist)

    def save_map(self):
        """
        Robotun haritasını kaydeder.
        """
        if not self.map_saved:  # Haritanın yalnızca bir kez kaydedilmesini sağlar
            rospy.loginfo("Harita kaydediliyor...")
            # Haritayı kaydetmek için sistem komutunu çalıştır
            os.system("rosrun map_server map_saver -f ~/catkin_ws/src/final_odevi/src/maps/map")
            rospy.loginfo("Harita kaydedildi.")
            self.map_saved = True  # Haritanın kaydedildiğini işaretle
            self.stop_robot()  # Harita kaydedildikten sonra robotu durdur

    def stop_robot(self):
        """
        Robotu durdurur ve ROS düğümünü kapatır.
        """
        rospy.loginfo("Robot durduruluyor.")
        self.twist.linear.x = 0.0  # İleri hareketi durdur
        self.twist.angular.z = 0.0  # Dönmeyi durdur
        self.cmd_pub.publish(self.twist)  # Hareket komutlarını yayınla
        rospy.signal_shutdown("Görev tamamlandı ve robot durduruldu.")  # ROS düğümünü sonlandır

    def run(self):
        """
        Robotun ana döngüsünü çalıştırır.
        """
        rospy.loginfo("Hareket başlatıldı.")
        while not rospy.is_shutdown():
            # Geçen süreyi hesapla
            current_time = rospy.Time.now()
            elapsed_time = (current_time - self.start_time).to_sec()

            if elapsed_time >= 77.0 and not self.map_saved:
                self.save_map()  # Haritayı kaydet

            self.rate.sleep()  # Döngü hızını koru

if __name__ == '__main__':
    try:
        # ObstacleAvoidance sınıfını başlat ve çalıştır
        obstacle_avoidance = ObstacleAvoidance()
        obstacle_avoidance.run()
    except rospy.ROSInterruptException:
        # ROS düğümü bir kesinti alırsa programı sonlandır
        pass

