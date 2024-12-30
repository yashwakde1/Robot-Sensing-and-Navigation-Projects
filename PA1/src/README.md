# EECE5554 ASSIGNMENT PA1 Yash Wakde

# Task 1

## step Write and test a publisher node that publishes a TOPIC and subscriber node that subscribes to this TOPIC. 
package name : py_pubsub
executable nodes : talker
executable nodes : listener 
## steps to execute in terminal 1 (publisher)
cd EECE5554/PA1/src
colcon build 
ros2 run py_pubsub talker 
output : [INFO] [1726672646.753035147] [minimal_publisher]: Publishing: "yash: 0"

## steps to execute in terminal 2 (subscriber)
ros2 run py_pubsub listener 
output : [INFO] [1726672716.213973186] [minimal_subscriber]: I heard: "yash: 0"



# Task 2

## Write and test a server node that provides a SERVICE and client node that calls this SERVICE
package name : py_srvcli
executable nodes :service
executable nodes :client

## steps to execute in terminal 1 (service)
cd EECE5554/PA1/src
colcon build 
ros2 run py_srvcli service
 output : ros2 run py_srvcli service
[INFO] [1726673533.671762181] [minimal_service]: Incoming request
a: 4 b: 6


## steps to execute in terminal 2 (client)
ros2 run py_srvcli client 4 6
.[INFO] [1726673533.680055762] [minimal_client_async]: Result of add_two_ints: for 4 + 6 = 10



