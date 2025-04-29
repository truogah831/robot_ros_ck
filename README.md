# ROS_CK_PROJECT: Karto_slam and Navigation for Robot

# Sau khi git clone package 

## 1. Khoi chay cac ban do voi Slam

- chay cac file launch tu map1_karto.launch den map6_karto.launch
    ```bash
  roslaunch robot_ros_gk map1_karto.launch
    ```
    ```bash
  roslaunch robot_ros_gk map2_karto.launch
    ```
    ```bash
  roslaunch robot_ros_gk map3_karto.launch
    ```
    ```bash
   roslaunch robot_ros_gk map4_karto.launch
    ```
    ```bash
  roslaunch robot_ros_gk map5_karto.launch
    ```
    ```bash
  roslaunch robot_ros_gk map6_karto.launch
    ```
#### Điều khiển robot di chuyển bằng bàn phím de quet ban do 
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
##2. Chạy file launch chay Navigation 

```bash
roslaunch robot_ros_gk navigation.launch
```
###Sau khi chay file navigation.launch can chon diem 2d pose estimate va 2d nav goal de robto thuc hien tinh toan va di chuyen











