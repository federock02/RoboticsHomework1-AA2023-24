Format: ROS Bag file
play the bag with the command:
rosbag play --clock robotics.bag

Data:
- /fix gps data
- /odom odometry from encoders
- /os_cloud_node/points pointcloud from lidar

*First Node*
  - Create a ROS package called first_project
  - Create a ROS node to compute the odometry from gps data:
    - write a node called gps_to_odom which publish:
    - odometry message:
      - type: nav_msgs/Odometry
      - topic name: gps_odom
  - to convert gps data to odometry:
    - convert (latitude-longitude-altitude) to Cartesian ECEF
    - convert Cartesian ECEF to ENU
    - ENU is a relative position, so you need to specify the reference point
  - gps_to_odom should have three parameters :
    - lat_r
    - lon_r
    - alt_r
  - These parameters should be set in a launch file, for this project you can set manually to the first value from GPS:
  - GPS gives you only position
  - For the odometry you also want orientation
  - In this scenario we work on a 2D plane, so we want the robot heading
  - After computing the robot position in ENU you can use consecutive poses to estimate the robot heading

*Second Node*
  - Inside the first_project package
  - Create a ROS node to convert from odometry to tf:
    - write a node called odom_to_tf which subscribe to:
      - odometry message:
        - type: nav_msgs/Odometry
        - topic name: input_odom (this is a generic name, you are supposed to use remapping to subscribe to the correct topic)
    - publish with a tf broadcaster an odometry between two values set as node parameters:
      - the two node parameters are called root_frame and child_frame
      - the node is supposed to be started from launch file with topic remapping for input and parameter for the tf_broardcaster
  - You should write a launch file which create two instances of this node, one to publish as a tf the odometry from the encoders, and one the odometry from gps
  - both published tf should have the same root set as world, but the child frame should be wheel_odom and gps_odom respectively

  *Third Node*
  - Inside the first_project package:
    - Final node for lidar data visualization (called lidar_remap)
    - The node subscribe to the /os_cloud_node/points topic and change the frame set in the header
    - The value set in the header is regulated by a dynamic reconfigure callback which allow to dynamically change it to be set to the wheel_odom or gps_odom frame
    - The final node should allow the user to select from rqt_reconfigure to which tf the lidar is connected
    - The node should publish on the topic /pointcloud_remapped
    - The three node should all start from a single launch file called launch.launch
    - The launch file should also open rviz
    - the only command to start the project should be: roslaunch first_project launch.launch
