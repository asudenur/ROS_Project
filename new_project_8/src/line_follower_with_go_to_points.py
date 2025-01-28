#!/usr/bin/env python3
import rospy
import os
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

class GoToPoints:
    def __init__(self):

        # Robotun hız komutları için publisher
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # LIDAR verisi için subscriber
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        # Odometre verisi için subscriber
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Hedef noktalar (x, y)
        self.waypoints = [
            {"x": 1.8, "y": 3.5},  # Hedef 1
            {"x": 0.5, "y": 1.5},  # Hedef 2
            {"x": 1.6, "y": 2.0},  # Hedef 3
        ]
        self.current_waypoint_index = 0

        # Robotun pozisyonu ve yönü
        self.current_position = None
        self.current_yaw = None

        # LIDAR engel tespiti
        self.obstacle_detected = False

        # Hareket komutları
        self.move_cmd = Twist()

    def laser_callback(self, msg):
        """
        LIDAR verilerini alır ve engel tespiti yapar.
        """
        try:
            ranges = msg.ranges
            front_range = min(ranges[0:30] + ranges[330:360])  # Ön taraftaki mesafeler
            self.obstacle_detected = front_range < 0.5  # Engel mesafesi (0.5 metre)
        except Exception as e:
            rospy.logerr(f"LIDAR hatası: {e}")

    def odom_callback(self, msg):
        """
        Robotun odometre verilerini alır.
        """
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        orientation = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

    def quaternion_to_yaw(self, x, y, z, w):
        """
        Kuaterniyonu yaw (z ekseni etrafında açı) değerine dönüştürür.
        """
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny, cosy)

    def has_reached_waypoint(self, waypoint):
        """
        Robotun hedef noktaya ulaşıp ulaşmadığını kontrol eder.
        """
        if self.current_position:
            distance = math.sqrt(
                (self.current_position[0] - waypoint["x"]) ** 2
                + (self.current_position[1] - waypoint["y"]) ** 2
            )
            return distance < 0.2  # Hedefe yakınlık
        return False

    def navigate_to_waypoint(self, waypoint):
        """
        Belirtilen hedef noktaya navigasyon yapar.
        """
        if self.current_position and self.current_yaw is not None:
            # Hedefe olan açı
            target_angle = math.atan2(
                waypoint["y"] - self.current_position[1],
                waypoint["x"] - self.current_position[0],
            )
            angle_diff = target_angle - self.current_yaw

            # Açıları normalize et (-pi ile pi arasında)
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            # Açı düzeltme
            if abs(angle_diff) > 0.1:  # Eğer hedefe doğru bakmıyorsa
                self.move_cmd.linear.x = 0.0
                self.move_cmd.angular.z = 0.3 if angle_diff > 0 else -0.3
            else:
                self.move_cmd.linear.x = 0.2  # Hedefe doğru ilerle
                self.move_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(self.move_cmd)

    def avoid_obstacle(self):
        """
        Engelden kaçınmak için robotu döndürür.
        """
        rospy.loginfo("Engel algılandı, kaçınma manevrası yapılıyor.")
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.5
        self.cmd_vel_pub.publish(self.move_cmd)
        rospy.sleep(1.5)  # Dönüş süresi

    def stop_robot(self):
        """
        Robotu durdurur.
        """
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(self.move_cmd)

    def execute(self):
        """
        Robotun hareket planını yürütür.
        """
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.current_waypoint_index < len(self.waypoints):
                current_waypoint = self.waypoints[self.current_waypoint_index]

                if self.obstacle_detected:
                    self.avoid_obstacle()
                elif not self.has_reached_waypoint(current_waypoint):
                    rospy.loginfo(f"Hedefe ilerleniyor: {current_waypoint}")
                    self.navigate_to_waypoint(current_waypoint)
                else:
                    rospy.loginfo(f"Hedefe ulaşıldı: {current_waypoint}")
                    self.stop_robot()
                    rospy.sleep(2)
                    self.current_waypoint_index += 1
            else:
                rospy.loginfo("Tüm hedeflere ulaşıldı.")
                self.stop_robot()
                break

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('line_follower_and_go_to_points', anonymous=True)
    line_follower = LineFollower()
    line_follower.move()  # Burada 'execute()' yerine 'move()' çağrılıyor.
    go_to_points = GoToPoints()
    go_to_points.execute()

