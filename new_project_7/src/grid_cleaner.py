#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class GridCleaner:
    def __init__(self):
        rospy.init_node('grid_cleaner', anonymous=True)

        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Robot state
        self.position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.scan_data = []
        
        # Movement parameters
        self.linear_speed = 0.15  # Reduced speed for better control
        self.angular_speed = 0.5
        self.stopping_distance = 0.4
        
        # Pattern parameters
        self.state = 'MOVE_FORWARD'
        self.turn_target = None
        self.start_position = None
        self.movement_distance = 0.5  # Distance to move in meters
        
        rospy.sleep(1)  # Wait for connections to establish

    def odom_callback(self, msg):
        # Get position
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        
        # Get orientation
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.position['theta'] = euler_from_quaternion(orientation_list)

    def laser_callback(self, msg):
        self.scan_data = msg.ranges

    def check_obstacle(self):
        if not self.scan_data:
            return False
        
        # Check front area (-30 to +30 degrees)
        front_ranges = []
        for i in range(-30, 31):
            idx = (i + 360) % 360
            if idx < len(self.scan_data) and not math.isnan(self.scan_data[idx]):
                front_ranges.append(self.scan_data[idx])
        
        if front_ranges:
            return min(front_ranges) < self.stopping_distance
        return False

    def move_forward(self):
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        self.cmd_vel_pub.publish(cmd)

    def turn(self, target_angle):
        current_angle = self.position['theta']
        angle_diff = target_angle - current_angle
        
        # Normalize angle difference to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        cmd = Twist()
        if abs(angle_diff) > 0.1:  # Threshold for angle precision
            cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            self.cmd_vel_pub.publish(cmd)
            return False
        return True

    def execute_movement(self):
        if self.state == 'MOVE_FORWARD':
            if self.check_obstacle():
                self.stop()
                self.state = 'TURN_RIGHT'
                self.turn_target = self.position['theta'] - math.pi/2  # 90 degrees right
            else:
                self.move_forward()

        elif self.state == 'TURN_RIGHT':
            if self.turn(self.turn_target):
                self.stop()
                self.state = 'MOVE_SIDE'
                self.start_position = self.position.copy()

        elif self.state == 'MOVE_SIDE':
            if self.check_obstacle():
                self.stop()
                self.state = 'TURN_RIGHT_1'
                self.turn_target = self.position['theta'] - math.pi/2
            else:
                dx = self.position['x'] - self.start_position['x']
                dy = self.position['y'] - self.start_position['y']
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance >= self.movement_distance:
                    self.stop()
                    self.state = 'TURN_RIGHT_1'
                    self.turn_target = self.position['theta'] - math.pi/2
                else:
                    self.move_forward()
        
        elif self.state == 'TURN_RIGHT_1':
            if self.turn(self.turn_target):
                self.stop()
                self.state = 'MOVE_DOWN'
                
        elif self.state == 'MOVE_DOWN':
            if self.check_obstacle():
                self.stop()
                self.state = 'TURN_LEFT'
                self.turn_target = self.position['theta'] + math.pi/2
            else:
                self.move_forward()
                     
        elif self.state == 'TURN_LEFT':
            if self.turn(self.turn_target):  # Turn left 
                self.stop()
                self.state = 'MOVE_SIDE_1'
                self.start_position = self.position.copy()

        # Fix for MOVE_SIDE_1 transition
        elif self.state == 'MOVE_SIDE_1':
            if self.check_obstacle():
                self.stop()
                self.state = 'TURN_LEFT_1'
                self.turn_target = self.position['theta'] + math.pi/2
            else:
                dx = self.position['x'] - self.start_position['x']
                dy = self.position['y'] - self.start_position['y']
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance >= self.movement_distance:
                    self.stop()
                    self.state = 'TURN_LEFT_1'
                    self.turn_target = self.position['theta'] + math.pi/2
                else:
                    self.move_forward()

        elif self.state == 'TURN_LEFT_1':
            if self.turn(self.turn_target):  # Turn left
                self.stop()
                self.state = 'MOVE_FORWARD'
 

        rospy.loginfo(f"State: {self.state}, Position: ({self.position['x']:.2f}, {self.position['y']:.2f}), "
                     f"Angle: {math.degrees(self.position['theta']):.1f}°")

    def stop(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        rospy.sleep(0.1)  # Short pause after stopping

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        rospy.loginfo("Starting movement...")
        
        while not rospy.is_shutdown():
            self.execute_movement()
            rate.sleep()

if __name__ == '__main__':
    try:
        cleaner = GridCleaner()
        cleaner.run()
    except rospy.ROSInterruptException:
        pass
