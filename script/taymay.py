#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import sys
import tty
import termios

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def arm_controller():
    rospy.init_node('arm_controller', anonymous=True)
    khau1_pub = rospy.Publisher('/khau1_link_joint/position', Float64, queue_size=10)  # Khớp tịnh tiến
    khau2_pub = rospy.Publisher('/khau2_link_joint/command', Float64, queue_size=10)  # Khớp quay
    
    # Khởi tạo vị trí của các khớp
    khau1_pos = 0.0  # Vị trí ban đầu của khâu tịnh tiến
    khau2_pos = 0.0  # Vị trí ban đầu của khâu quay
    
    # Đặt bước di chuyển cho mỗi khớp
    step1 = 0.01  # Bước cho khâu tịnh tiến
    step2 = 0.02  # Bước cho khâu quay
    
    print("\nĐiều khiển tay máy:")
    print("1: Tăng vị trí khâu 1 (tịnh tiến)")
    print("2: Giảm vị trí khâu 1 (tịnh tiến)")
    print("3: Tăng vị trí khâu 2 (quay)")
    print("4: Giảm vị trí khâu 2 (quay)")
    print("q: Thoát")
    
    rate = rospy.Rate(50)  # Tần số xuất bản là 50Hz
    
    while not rospy.is_shutdown():
        key = get_key()
        
        if key == '1':
            khau1_pos += step1  # Tăng vị trí khâu 1
            if khau1_pos > 0.1:  # Giới hạn của khâu tịnh tiến
                khau1_pos = 0.1
        elif key == '2':
            khau1_pos -= step1  # Giảm vị trí khâu 1
            if khau1_pos < 0:  # Giới hạn của khâu tịnh tiến
                khau1_pos = 0
        elif key == '3':
            khau2_pos += step2  # Tăng vị trí khâu 2
            if khau2_pos > 1.57:  # Giới hạn trên của khâu quay
                khau2_pos = 1.57
        elif key == '4':
            khau2_pos -= step2  # Giảm vị trí khâu 2
            if khau2_pos < -0.3:  # Giới hạn dưới của khâu quay
                khau2_pos = -0.3
        elif key == 'q':
            break  # Thoát chương trình
        
        # Xuất bản các vị trí mới cho các khớp
        khau1_pub.publish(khau1_pos)
        khau2_pub.publish(khau2_pos)
        
        rospy.loginfo("Khâu 1: %.2f m, Khâu 2: %.2f rad", khau1_pos, khau2_pos)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        arm_controller()
    except rospy.ROSInterruptException:
        pass
