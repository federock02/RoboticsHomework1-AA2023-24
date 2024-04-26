#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "math.h"

double lat;
double lon;
double alt;

// Extract the latitude, longitude, and altitude values from the NavSatFix message from the /fix topic
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    lat = msg->latitude;
    lon = msg->longitude;
    alt = msg->altitude;
}

double computeN(double lat, double e2, double a)
{
    return N = a / sqrt(1 - (e2 * (sin(lat) * sin(lat)))); // prime vertical radius of curvature (distance from the surface to the Z-axis along the ellipsoid normal)
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gps_to_odom");
    ros::NodeHandle nh;

    // publish the Odometry message to the /gps_odom topic
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 1);
    
    // subscribe to the /fix topic from the robotics.bag file
    // and extract the latitude, longitude, and altitude values
    // from the NavSatFix message
    ros::Subscriber sub = nh.subscribe("/fix", 1, gpsCallback);

    // set the loop rate to 1 Hz
    ros::Rate loop_rate(1);

    double old_enu[] = {0.0, 0.0, 0.0};
    double ecef[3];
    double enu[3];
    double delta_ECEF[3];
    double translation[3];
    double pitch;
    double yaw;
    double roll;
    double orientation_quaternion[4];
    double lla_rad[3];

    // paramethers for the WGS84 ellipsoid
    double a = 6378137.0; // semi-major axis of the Earth
    double b = 6356752.3; // semi-minor axis of the Earth
    double e2 = 1 - ((b * b) / (a * a)); // eccentricity of the Earth

    // get reference parameters from launch file
    double lat_r;
    double lon_r;
    double alt_r;
    nh.param("/lat_r", lat_r, 1.0);
    nh.param("/lon_r", lon_r, 1.0);
    nh.param("/alt_r", alt_r, 1.0);
    lat_r = lat_r * (M_PI/180);
    lon_r = lon_r * (M_PI/180);
    alt_r = alt_r;

    // Convert Cartesian LLA to ECEF for reference point
    double N_ref = computeN(lat_r, e2, a);
    double ecef_r[3];
    ecef_r[0] = (N_ref + alt_r) * cos(lat_r) * cos(lon_r);
    ecef_r[1] = (N_ref + alt_r) * cos(lat_r) * sin(lon_r);
    ecef_r[2] = (N_ref * (1 - e2) + alt_r) * sin(lat_r);
    ROS_INFO("ECEF REF: x = %f, y = %f, z = %f", ecef_r[0], ecef_r[1], ecef_r[2]);

    // loop until ROS is shutdown
    while (ros::ok())
    {
        // create an Odometry message
        nav_msgs::Odometry odom_msg;
        
        lla_rad[0] = lat * (M_PI/180);
        lla_rad[1] = lon * (M_PI/180);
        lla_rad[2] = alt;

        // Convert Cartesian LLA to ENU
        double N = computeN(lla_rad[0], e2, a);
        ecef[0] = (N + alt) * cos(lat) * cos(lon);
        ecef[1] = (N + alt) * cos(lat) * sin(lon);
        ecef[2] = (N * (1 - e2) + alt) * sin(lat);
        ROS_INFO("ECEF: x = %f, y = %f, z = %f", ecef[0], ecef[1], ecef[2]);
        
        // Convert Cartesian ECEF to ENU
        delta_ECEF[0] = ecef[0] - ecef_r[0];
        delta_ECEF[1] = ecef[1] - ecef_r[1];
        delta_ECEF[2] = ecef[2] - ecef_r[2];
        enu[0] = - sin(ecef_r[0]) * delta_ECEF[0] + cos(ecef_r[0]) * delta_ECEF[1];
        enu[1] = - sin(ecef_r[1]) * cos(ecef_r[0]) * delta_ECEF[0] - sin(ecef_r[1]) * sin(ecef_r[0]) * delta_ECEF[1] + cos(ecef_r[1]) * delta_ECEF[2];
        // double up = cos(ecef_r[0]) * cos(ecef_r[1]) * delta_ECEF[0] + cos(ecef_r[0]) * sin(ecef_r[1]) * delta_ECEF[1] + sin(ecef_r[0]) * delta_ECEF[2];
        enu[2] = 0.0; // keep everything in 2D

        // Populate the Odometry message
        odom_msg.pose.pose.position.x = enu[0];
        odom_msg.pose.pose.position.y = enu[1];
        odom_msg.pose.pose.position.z = enu[2];

        // Calculate the orientation of the robot in the ENU frame, using consecutive poses estimation
        translation[0] = enu[0] - old_enu[0];
        translation[1] = enu[1] - old_enu[1];
        translation[2] = enu[2] - old_enu[2];
        old_enu[0] = enu[0];
        old_enu[1] = enu[1];
        old_enu[2] = enu[2];
        
        // pitch = atan(translation[2] / translation[1]) * (180/M_PI);
        pitch = 0.0;
        yaw = atan(translation[1] / translation[2]);
        // roll = atan(translation[0] / translation[2]) * (180/M_PI);
        roll = 0.0;

        // Convert the Euler angles to a quaternion
        double cy = cos(yaw / 2);
        double sy = sin(yaw / 2);
        double cp = cos(pitch / 2);
        double sp = sin(pitch / 2);
        double cr = cos(roll / 2);
        double sr = sin(roll / 2);

        orientation_quaternion[0] = cy * cp * sr - sy * sp * cr;
        orientation_quaternion[1] = sy * cp * sr + cy * sp * cr;
        orientation_quaternion[2] = sy * cp * cr - cy * sp * sr;
        orientation_quaternion[3] = cy * cp * cr + sy * sp * sr;

        odom_msg.pose.pose.orientation.x = orientation_quaternion[0];
        odom_msg.pose.pose.orientation.y = orientation_quaternion[1];
        odom_msg.pose.pose.orientation.z = orientation_quaternion[2];
        odom_msg.pose.pose.orientation.w = orientation_quaternion[3];

        pub.publish(odom_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}