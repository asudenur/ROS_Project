#!/usr/bin/env python3
import rospy
import os  # Harita kaydetmek için gerekli
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class LineFollower:
    def __init__(self):
        # Hareket komutlarını yayınlamak için publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # LaserScan verilerini alacak subscriber
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        # Odometriden pozisyon verilerini almak için subscriber
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Hareket komutlarını tanımlamak
        self.move_cmd = Twist()

        # Robotun hız parametreleri
        self.linear_speed = 0.2  # İleri doğru hız
        self.angular_speed = 0.5  # Dönüş hızı

        # Engelleme durumu
        self.obstacle_detected = False

        # Harita kaydetme durumu
        self.map_saved = False

        # Başlangıç pozisyonunu ve hareket durumu
        self.start_position = None
        self.current_position = None
        self.distance_travelled = 0.0
        self.has_moved = False

    def laser_callback(self, msg):
        try:
            # Ön tarafa odaklanarak mesafeleri kontrol et
            ranges = msg.ranges
            front_range = min(ranges[0:30] + ranges[330:360])

            # Engel tespiti
            self.obstacle_detected = front_range < 0.5
        except Exception as e:
            rospy.logerr(f"Laser callback hatası: {e}")

    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        # Başlangıç pozisyonunu kaydet
        if self.start_position is None:
            self.start_position = self.current_position

        # Hareket edilen toplam mesafeyi hesapla
        if self.current_position and self.start_position:
            self.distance_travelled = math.sqrt(
                (self.current_position[0] - self.start_position[0]) ** 2 +
                (self.current_position[1] - self.start_position[1]) ** 2
            )
            # Robot başlangıç noktasından hareket ettiyse işaretle
            if self.distance_travelled > 0.5:  # 0.5 metre ilerleme toleransı
                self.has_moved = True

    def save_map(self):
        """
        Robotun haritasını kaydeder.
        """
        if not self.map_saved:
            rospy.loginfo("Harita kaydediliyor...")
            try:
                os.system("rosrun map_server map_saver -f ~/catkin_ws/src/but_odevi/src/maps/map")
                rospy.loginfo("Harita kaydedildi.")
                self.map_saved = True
                rospy.signal_shutdown("Görev tamamlandı ve robot durduruldu.")
            except Exception as e:
                rospy.logerr(f"Harita kaydedilirken hata: {e}")

    def stop_robot(self):
        """
        Robotu durdurur.
        """
        rospy.loginfo("Robot durduruluyor.")
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)

    def has_returned_to_start(self):
        """
        Robotun başlangıç pozisyonuna dönüp dönmediğini kontrol eder.
        """
        if self.start_position and self.current_position:
            distance = math.sqrt(
                (self.current_position[0] - self.start_position[0]) ** 2 +
                (self.current_position[1] - self.start_position[1]) ** 2
            )
            return distance < 0.2 and self.has_moved  # Hareket ettiyse ve başlangıca yaklaştıysa
        return False

    def rotate_degrees(self, degrees):
        """
        Robotu belirtilen derece kadar döndürür.
        """
        rospy.loginfo(f"Robot {degrees} derece döndürülüyor...")
        angular_speed = self.angular_speed
        turn_duration = math.radians(degrees) / angular_speed
        end_time = rospy.Time.now() + rospy.Duration(turn_duration)
        while rospy.Time.now() < end_time:
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = angular_speed
            self.cmd_vel_pub.publish(self.move_cmd)
        self.move_cmd.angular.z = 0.0  # Dönüşü durdur
        self.cmd_vel_pub.publish(self.move_cmd)

    def move(self):
        rospy.init_node('line_follower', anonymous=True)
        rate = rospy.Rate(10)

        try:
            going_forward = True
            while not rospy.is_shutdown():
                if not self.has_moved:
                    rospy.loginfo("Robot başlangıç noktasından hareket ediyor...")
                    self.move_cmd.linear.x = self.linear_speed
                    self.move_cmd.angular.z = 0.0
                elif self.obstacle_detected and going_forward:
                    rospy.loginfo("Engel tespit edildi, geri dönüş başlatılıyor...")
                    self.rotate_degrees(360)  # İlk olarak 360 derece dön
                    self.rotate_degrees(180)  # Ardından 180 derece dön
                    going_forward = False

                if self.has_returned_to_start():
                    rospy.loginfo("Başlangıç noktasına dönüldü.")
                    self.stop_robot()  # Robotu durdur
                    rospy.sleep(2)  # Haritayı kaydetmeden önce kısa bir süre bekle
                    self.save_map()
                    break

                if going_forward:
                    # Engel yoksa düz ilerle
                    self.move_cmd.linear.x = self.linear_speed
                    self.move_cmd.angular.z = 0.0
                else:
                    # Geri dön
                    self.move_cmd.linear.x = self.linear_speed
                    self.move_cmd.angular.z = 0.0

                # Hareket komutlarını yayınla
                self.cmd_vel_pub.publish(self.move_cmd)

                rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS düğümü durduruldu.")
        except Exception as e:
            rospy.logerr(f"Main loop hatası: {e}")


if __name__ == '__main__':
    try:
        line_follower = LineFollower()
        line_follower.move()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node kapatıldı.")

