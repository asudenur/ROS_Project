#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time
import sys
import termios
import tty

def get_key():
    """Klavye girdisini almak için fonksiyon."""
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    try:
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def move():
    # ROS node'unu başlat
    rospy.init_node('move', anonymous=True)

    # Hareket komutları için publisher
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Hareket mesajı
    move_cmd = Twist()

    rate = rospy.Rate(10)  # 10 Hz

    # 1. 5 saniye ileri git
    move_cmd.linear.x = 0.2  # İleri hareket için hızı ayarla
    move_cmd.angular.z = 0.0
    start_time = rospy.Time.now().to_sec()
    rospy.loginfo("Robot ileri gidiyor...")
    while rospy.Time.now().to_sec() - start_time < 5:
        cmd_vel_pub.publish(move_cmd)
        rate.sleep()

    # 2. 90 derece sola dön
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.5  # Sola dönüş hızı
    start_time = rospy.Time.now().to_sec()
    rospy.loginfo("Robot 90 derece sola dönüyor...")
    while rospy.Time.now().to_sec() - start_time < (90 / (move_cmd.angular.z * 50)):
        cmd_vel_pub.publish(move_cmd)
        rate.sleep()

    # 3. 5 saniye geri git
    move_cmd.linear.x = -0.2  # Geri hareket için hızı ayarla
    move_cmd.angular.z = 0.0
    start_time = rospy.Time.now().to_sec()
    rospy.loginfo("Robot geri gidiyor...")
    while rospy.Time.now().to_sec() - start_time < 5:
        cmd_vel_pub.publish(move_cmd)
        rate.sleep()

    # 4. 90 derece sağa dön
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = -0.5  # Sağa dönüş hızı
    start_time = rospy.Time.now().to_sec()
    rospy.loginfo("Robot 90 derece sağa dönüyor...")
    while rospy.Time.now().to_sec() - start_time < (90 / (abs(move_cmd.angular.z) * 50)):
        cmd_vel_pub.publish(move_cmd)
        rate.sleep()

    # 5. 5 saniye ileri git
    move_cmd.linear.x = 0.2  # İleri hareket için hızı ayarla
    move_cmd.angular.z = 0.0
    start_time = rospy.Time.now().to_sec()
    rospy.loginfo("Robot ileri gidiyor...")
    while rospy.Time.now().to_sec() - start_time < 5:
        cmd_vel_pub.publish(move_cmd)
        rate.sleep()

    # 6. 135 derece sola dön
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.5  # Dönüş hızı, 0.5 rad/s
    duration = 135 / (move_cmd.angular.z * (180 / 3.14159))  # Dönüş açısını radiana çevir
    start_time = rospy.Time.now().to_sec()
    rospy.loginfo("Robot 135 derece sola dönüyor...")
    while rospy.Time.now().to_sec() - start_time < duration:
        cmd_vel_pub.publish(move_cmd)
        rate.sleep()

    # 7. 5 saniye daha ileri git
    move_cmd.linear.x = 0.2
    move_cmd.angular.z = 0.0
    start_time = rospy.Time.now().to_sec()
    rospy.loginfo("Robot ileri gidiyor...")
    while rospy.Time.now().to_sec() - start_time < 5:
        cmd_vel_pub.publish(move_cmd)
        rate.sleep()

    # Hareketi durdur
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    cmd_vel_pub.publish(move_cmd)
    rospy.loginfo("Hareket Tamamlandı.")

    rospy.loginfo("Klavye kontrolü için 'W' (ileri), 'S' (geri), 'A' (sola), 'D' (sağa) tuşlarına basın, 'Q' ile çıkın.")

    while not rospy.is_shutdown():
        key = get_key()

        if key == 'w':
            move_cmd.linear.x = 0.2  # İleri
            move_cmd.angular.z = 0.0
            print("[HAREKET] İleri gidiliyor...")
        elif key == 's':
            move_cmd.linear.x = -0.2  # Geri
            move_cmd.angular.z = 0.0
            print("[HAREKET] Geri gidiliyor...")
        elif key == 'a':
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.5  # Sola döndür
            print("[HAREKET] Sola dönülüyor...")
        elif key == 'd':
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = -0.5  # Sağa döndür
            print("[HAREKET] Sağa dönülüyor...")
        elif key == 'q':
            print("[HAREKET] Çıkılıyor...")
            break  # Çıkmak için
        else:
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0

        cmd_vel_pub.publish(move_cmd)
        rate.sleep()

    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    cmd_vel_pub.publish(move_cmd)
    rospy.loginfo("Robot Durdu.")

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
