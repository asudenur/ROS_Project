#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math


class SimpleNavigator:
    def __init__(self):
        rospy.init_node("simple_navigator", anonymous=True)

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


if __name__ == "__main__":
    try:
        navigator = SimpleNavigator()
        navigator.execute()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigasyon düğümü sonlandırıldı.")

