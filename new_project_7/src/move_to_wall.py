#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan  
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry 
import tf.transformations 
import math

class MoveToWall:
    def __init__(self):
        # ROS düğümünü başlat
        rospy.init_node('move_to_wall', anonymous=True)

        # Hareket komutları için publisher oluştur
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Lidar verilerini dinlemek için subscriber oluştur
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Odometry verilerini dinlemek için subscriber oluştur
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Hareket komutu nesnesi (Twist mesajı)
        self.move_cmd = Twist()

        # Lidar verilerinden en yakın duvarın bilgileri
        self.closest_distance = float('inf')  # Başlangıçta mesafe sonsuz kabul edilir
        self.closest_angle = 0  # En yakın duvarın açısı

        # Lidar taramasının tamamlanıp tamamlanmadığını kontrol etmek için değişken
        self.scan_complete = False

        # Robotun yön bilgileri
        self.start_yaw = None  # Taramanın başladığı yön
        self.current_yaw = None  # Robotun mevcut yönü

        # Döngü frekansı (10 Hz)
        self.rate = rospy.Rate(10)

    def scan_callback(self, scan_data):
        """
        Lidar verilerini analiz eder ve en kısa mesafeyi bulur.
        """
        # Eğer tarama tamamlanmamışsa, en kısa mesafeyi belirle
        if not self.scan_complete:
            for i, distance in enumerate(scan_data.ranges):
                if 0.1 < distance < self.closest_distance:  # Geçersiz verileri filtrele
                    self.closest_distance = distance  # En kısa mesafeyi kaydet
                    self.closest_angle = i  # Bu mesafeye ait açıyı kaydet
        else:
            # Tarama tamamlandıktan sonra da en kısa mesafeyi güncelle
            self.closest_distance = min(scan_data.ranges)

    def odom_callback(self, odom_data):
        """
        Odometry verilerinden robotun mevcut yönünü belirler.
        """
        orientation_q = odom_data.pose.pose.orientation  # Quaternion verisi
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.current_yaw) = tf.transformations.euler_from_quaternion(orientation_list)

    def rotate_and_scan(self):
        """
        Robotun tam bir dönüş yaparak çevresini taramasını sağlar.
        """
        rospy.loginfo("Tarama başlıyor...")

        self.move_cmd.angular.z = 0.3  # Robotun yavaşça dönmesini sağla

        # Dönüşün başlangıç zamanını kaydet
        self.start_yaw = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.move_cmd)  # Dönme komutunu gönder
            current_yaw = rospy.Time.now().to_sec()

            # Robot tam bir dönüş yaptığında işlemi sonlandır
            if (current_yaw - self.start_yaw) >= (2 * math.pi / self.move_cmd.angular.z):
                rospy.loginfo("Tarama tamamlandı. En yakın mesafe: %.2f metre, açı: %d derece" % (
                    self.closest_distance, self.closest_angle))
                self.move_cmd.angular.z = 0.0  # Dönmeyi durdur
                self.cmd_vel_pub.publish(self.move_cmd)
                self.scan_complete = True  # Taramanın tamamlandığını işaretle
                break

            self.rate.sleep()

    def move_to_closest_wall(self):
        """
        Robotun en yakın duvara güvenli bir mesafeye kadar hareket etmesini sağlar.
        """
        if self.closest_distance == float('inf'):
            rospy.logwarn("Yakın bir hedef bulunamadı!")  # Eğer geçerli bir hedef yoksa uyarı ver
            return

        rospy.loginfo("En yakın duvara ilerleniyor...")
        target_distance = self.closest_distance - SAFE_DISTANCE  # Güvenli mesafeyi hesapla

        # Önce en yakın duvar yönüne dön
        angle_to_turn = math.radians(self.closest_angle)  # Açıyı dereceden radyana çevir
        if angle_to_turn > math.pi:
            angle_to_turn -= 2 * math.pi  # Açıyı [-π, π] aralığına getir

        rospy.loginfo("Hedef açı: %.2f radyan" % angle_to_turn)

        # Robotun hedef açıya dönmesi için gerekli süreyi hesapla
        self.move_cmd.angular.z = 0.2 if angle_to_turn > 0 else -0.2
        turn_duration = abs(angle_to_turn) / 0.2
        turn_start_time = rospy.Time.now()

        while (rospy.Time.now() - turn_start_time).to_sec() < turn_duration:
            self.cmd_vel_pub.publish(self.move_cmd)
            self.rate.sleep()

        self.move_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(self.move_cmd)

        # Hedef duvara doğru ilerle
        self.move_cmd.linear.x = 0.2  # İleri hareket komutu

        while not rospy.is_shutdown():
            if self.closest_distance <= SAFE_DISTANCE:  # Güvenli mesafeye ulaşıldığında dur
                rospy.loginfo("Duvara ulaşıldı. Robot duruyor.")
                self.move_cmd.linear.x = 0.0
                self.cmd_vel_pub.publish(self.move_cmd)
                rospy.signal_shutdown("Görev tamamlandı. Robot duvara ulaştı.")
                break

            self.cmd_vel_pub.publish(self.move_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    SAFE_DISTANCE = 0.5  # Duvarla güvenli mesafe (metre)

    try:
        robot = MoveToWall()
        robot.rotate_and_scan()  # Çevresini tarar
        if robot.scan_complete:
            robot.move_to_closest_wall()  # En yakın duvara doğru ilerler
    except rospy.ROSInterruptException:
        pass

