# EECE5554 LAB1 Yash Wakde 


# Task 1
steps
Created a package named gps_data 

inside gps_drive/gps_driver/ driver.py file 

made a launch file to launch the executable

package="gps_driver",

executable="driver",


# Task 2

steps 
created a msg package named msg_package 

later created a folder named msg 

msg_package/msg/GPSmsg.msg is created 

updated all the .xml and cmake 

## Execute in terminal 1

- ros2 launch gps_driver gps_launch.launch.py

## Execute in terminal 2

- ros2 bag record /gps -o walking_data.bag
- ros2 bag record /gps -o stationary_data.bag

## To check topic is publishe 

- ros2 topic echo /gps

# Task 3 

made a analysis script  name analysis.py 

## Note :  I have run the code i.e Lab1_ros2_struct_checker and got no errors and got the screen values and the msg repository structure is correct .by 
## I have pushed the src as per mentioned in submission but, It might give an error while running the Lab1_ros2_struct_checker as it has files other than required so, please run Lab1_ros2_struct_checker after deleting files like rosbag, analysis scripts . 

## structure followed as mentioned in How to submit Lab1

- src (directory)

 - gps_driver (ROS2 package)
 - gps_driver (Python package with driver.py file)
 - launch (directory with the driver.launch file)
 - package.xml and other files present in your original workspace
 - ROS2 package for custom interface (msg)
 - Report.pdf (file)
 - Analysis scripts
 - data 