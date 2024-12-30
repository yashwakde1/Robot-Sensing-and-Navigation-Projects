# EECE5554 LAB4 Yash Wakde 


# Task 1
steps
Created a package named rsn_driver 

 which launches both gps driver and imu driver 

# Task 2


## Execute in terminal 1

- ros2 launch rsn_driver rsn_drivers_launch.py imu_port:=/dev/ttyUSB0 gps_port:=/dev/ttyUSB1 imu_baudrate:="115200" imu_frequency:=40

## Execute in terminal 2

- ros2 bag record /imu -o data_going_in_circles
- ros2 bag record /imu -o driving_data.bag

## Link to 5hrs Rosbag 
https://drive.google.com/drive/folders/1UQg1TbD0LVToMseAdrmUiCTDCTGcloT4?usp=sharing

data is in src folder 
     src 
          driving_data
          data_going_in_circles


## data was collected by my temate dhy mistry     gitlab username -  mistry.dhy

# Task 2

made a analysis script  name analysis.py 

## structure followed as mentioned in How to submit Lab4 

