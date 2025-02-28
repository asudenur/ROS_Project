#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from time import sleep


class RobotNavigator:
    def __init__(self):
        rospy.init_node("robot_navigation")
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        rospy.loginfo("Move_base sunucusu bekleniyor...")
        self.client.wait_for_server()
        rospy.loginfo("Move_base sunucusuna bağlanıldı!")
        self.scan_data = None
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.laser_callback)

    def laser_callback(self, msg):
        self.scan_data = msg.ranges

    def is_path_clear(self):
        if self.scan_data:
            # Ön taraftaki engelleri kontrol et
            front_ranges = self.scan_data[len(self.scan_data) // 3 : 2 * len(self.scan_data) // 3]
            min_distance = min(front_ranges)
            rospy.loginfo(f"Ön taraftaki en yakın engel mesafesi: {min_distance}")
            return min_distance > 0.5  # Engellerden uzaklık (ör. 0.5m)
        return True

    def move_to_goal(self, x_goal, y_goal):
        rospy.loginfo(f"Hedefe gidiliyor: x={x_goal}, y={y_goal}")

        # Hedef belirleme
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(x_goal, y_goal, 0)
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Hedef gönderiliyor...")
        self.client.send_goal(goal)

        while not self.client.wait_for_result(rospy.Duration(1)):
            if not self.is_path_clear():
                rospy.logwarn("Engel algılandı! Alternatif bir yol aranıyor...")
                self.client.cancel_goal()
                rospy.loginfo("5 saniye boyunca yeniden planlama bekleniyor...")
                sleep(5)  # Alternatif bir yol için bekleme süresi
                self.client.send_goal(goal)  # Aynı hedefi yeniden gönder
                continue

        if self.client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("Hedefe başarıyla ulaşıldı!")
            return True
        else:
            rospy.loginfo("Hedefe ulaşılamadı.")
            return False

    def scan_area(self):
        rospy.loginfo("Tarama yapılıyor...")
        sleep(5)  # Tarama simülasyonu için bekleme süresi
        rospy.loginfo("Tarama tamamlandı.")


if __name__ == "__main__":
    navigator = RobotNavigator()

    # Robotun ziyaret edeceği noktalar
    goals = [
        {"x": -3.0, "y": 4.0},
        {"x": 1.5, "y": 3.0},
        {"x": 3.0, "y": 2.0},
    ]

    for goal in goals:
        rospy.loginfo(f"Yeni hedef: x={goal['x']}, y={goal['y']}")
        success = False
        attempts = 3  # Hedef için maksimum deneme sayısı

        while not success and attempts > 0:
            success = navigator.move_to_goal(goal["x"], goal["y"])
            if not success:
                rospy.logwarn("Hedefe ulaşılamadı, tekrar deneniyor...")
                attempts -= 1

        if success:
            navigator.scan_area()  # Hedefe ulaşıldığında tarama yap
        else:
            rospy.logerr("Hedefe ulaşılamadı, bir sonraki hedefe geçiliyor.")

