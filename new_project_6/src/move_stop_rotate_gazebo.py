#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node')
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.cmd = Twist()
        self.min_distance = 0.5  # Minimum distance to stop
        self.obstacle_detected = False

    def laser_callback(self, scan_data):
        # Evaluate a wider range in front of the robot
        front_range = list(scan_data.ranges[330:359]) + list(scan_data.ranges[0:30])
        front_distance = min(front_range)

        rospy.loginfo(f"Front distance: {front_distance}")

        if front_distance < self.min_distance:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def move_forward(self):
        rospy.loginfo("Moving forward")
        self.cmd.linear.x = 0.2  # Move forward
        self.cmd.angular.z = 0.0  # No rotation
        self.velocity_publisher.publish(self.cmd)

    def stop_and_rotate(self):
        rospy.loginfo("Obstacle detected! Stopping and rotating.")
        # Stop the robot
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.velocity_publisher.publish(self.cmd)
        rospy.sleep(1)

        # Rotate 90 degrees
        self.cmd.angular.z = 0.5  # Angular speed
        self.cmd.linear.x = 0.0
        self.velocity_publisher.publish(self.cmd)
        rospy.sleep(2)  # Rotate for 2 seconds

        # Stop rotation
        self.cmd.angular.z = 0.0
        self.velocity_publisher.publish(self.cmd)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.obstacle_detected:
                self.stop_and_rotate()
            else:
                self.move_forward()
            rate.sleep()

if __name__ == '__main__':
    try:
        robot = ObstacleAvoidance()
        robot.run()
    except rospy.ROSInterruptException:
        pass

