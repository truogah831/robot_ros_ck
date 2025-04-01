# ROS_GK_PROJECT: BUILD AND CONTROL CAR-LIKE ROBOT WITH MANIPULATOR

# Các Thư Viện và Gói Cần Cài Đặt

Để chạy file URDF và launch file trong ROS, bạn cần cài đặt các thư viện và gói sau đây:

## 1. Các gói ROS cần thiết

- **robot_state_publisher**: Gói để xuất bản trạng thái của các liên kết robot.
  - Cài đặt:
    ```bash
    sudo apt-get install ros-noetic-robot-state-publisher
    ```

- **controller_manager**: Gói quản lý các controller cho robot.
  - Cài đặt:
    ```bash
    sudo apt-get install ros-noetic-controller-manager
    ```

- **joint_state_publisher_gui**: Gói để xuất bản và điều chỉnh các trạng thái của các khớp.
  - Cài đặt:
    ```bash
    sudo apt-get install ros-noetic-joint-state-publisher-gui
    ```

- **teleop_twist_keyboard**: Gói để điều khiển robot thông qua bàn phím.
  - Cài đặt:
    ```bash
    sudo apt-get install ros-noetic-teleop-twist-keyboard
    ```

- **ros_control**: Gói để điều khiển robot thông qua các interface.
  - Cài đặt:
    ```bash
    sudo apt-get install ros-noetic-ros-control
    sudo apt-get install ros-noetic-gazebo-ros-control
    ```

- **libgazebo_ros_control**: Plugin để tích hợp các controller của ROS với Gazebo.
  - Cài đặt:
    ```bash
    sudo apt-get install ros-noetic-libgazebo-ros-control
    ```

#### Sau khi git clone 
```bash
catkin_make
```
```bash
source devel/setup.bash
```
### 2. Chạy file launch để khởi động GAZEBO và RVIZ

```bash
roslaunch robot_ros_gk mophong.launch
```
#### Chạy mô phỏng và xem thông tin GPS
```bash
rostopic echo /gps/fix
```
#### Điều khiển robot di chuyển bằng bàn phím
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
#### Điều khiển tay máy: Khâu 1
```bash
rostopic pub /khau1_joint_position_controller/command std_msgs/Float64 "data: 0.1"
```
#### Giới hạn khâu 1: từ 0 đến 0.1

#### Điều khiển tay máy: Khâu 2
```bash
rostopic pub /khau2_joint_position_controller/command std_msgs/Float64 "data: 0.5"
```
#### Giới hạn khâu 2: từ -0.3 đến 1.57










