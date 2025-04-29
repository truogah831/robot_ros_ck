#!/usr/bin/env python

import rospy
import random
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class AutoExplore:
    def __init__(self):
        # Khởi tạo node ROS
        rospy.init_node('auto_explore', anonymous=True)

        # Publisher để điều khiển robot (topic /cmd_vel)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber để nhận dữ liệu LiDAR (topic /scan)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Publisher để giả lập joint calibration
        self.calibrated_pub = rospy.Publisher('/calibrated', Bool, queue_size=10)

        # Các tham số
        self.linear_speed = 0.2  # Tốc độ thẳng (m/s)
        self.angular_speed = 0.5  # Tốc độ góc (rad/s)
        self.min_distance = 0.5  # Khoảng cách tối thiểu để phát hiện chướng ngại vật (m)
        self.state = 'forward'  # Trạng thái: 'forward' (đi thẳng) hoặc 'turning' (quay)
        self.turn_duration = 0  # Thời gian quay (s)
        self.turn_start_time = 0  # Thời gian bắt đầu quay
        self.rate = rospy.Rate(10)  # Tần số lặp (Hz)

        # Biến lưu trữ dữ liệu LiDAR
        self.laser_ranges = []

        # Gửi tín hiệu calibrated
        self.calibrated_pub.publish(Bool(data=True))

    def scan_callback(self, data):
        """Callback để xử lý dữ liệu LiDAR"""
        self.laser_ranges = data.ranges

    def is_obstacle_ahead(self):
        """Kiểm tra xem có chướng ngại vật phía trước không"""
        if not self.laser_ranges:
            return False

        # Kiểm tra khoảng cách phía trước (góc 0 độ, tức là khoảng giữa của LiDAR)
        front_range = min(self.laser_ranges[len(self.laser_ranges)//2 - 10 : len(self.laser_ranges)//2 + 10])
        if front_range < self.min_distance and front_range > 0.0:  # Bỏ qua giá trị không hợp lệ (0 hoặc inf)
            return True
        return False

    def explore(self):
        """Chạy vòng lặp điều khiển robot tự động di chuyển"""
        while not rospy.is_shutdown():
            twist = Twist()

            if self.state == 'forward':
                # Di chuyển thẳng
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0

                # Nếu phát hiện chướng ngại vật, chuyển sang trạng thái quay
                if self.is_obstacle_ahead():
                    self.state = 'turning'
                    self.turn_start_time = rospy.get_time()
                    # Chọn ngẫu nhiên hướng quay (trái hoặc phải)
                    self.turn_direction = random.choice([-1, 1])  # -1: quay phải, 1: quay trái
                    self.turn_duration = random.uniform(1.0, 3.0)  # Quay trong khoảng 1-3 giây
                    rospy.loginfo("Obstacle detected, turning %s", "left" if self.turn_direction == 1 else "right")

            elif self.state == 'turning':
                # Quay tại chỗ
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed * self.turn_direction

                # Nếu đã quay đủ thời gian, chuyển lại trạng thái đi thẳng
                if rospy.get_time() - self.turn_start_time > self.turn_duration:
                    self.state = 'forward'
                    rospy.loginfo("Finished turning, moving forward")

            # Xuất bản vận tốc
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        explorer = AutoExplore()
        explorer.explore()
    except rospy.ROSInterruptException:
        pass
