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

void computeECEF(double ecef[], double lat, double lon, double alt)
{
    double a = 6378137.0; // semi-major axis of the Earth
    double b = 6356752.3; // semi-minor axis of the Earth
    double e2 = 1 - ((b * b) / (a * a)); // eccentricity of the Earth
    double N = a / sqrt(1 - (e2 * (sin(lat) * sin(lat)))); // prime vertical radius of curvature (distance from the surface to the Z-axis along the ellipsoid normal)
    // Convert latitude, longitude, and altitude to Cartesian ECEF
    double x = (N + alt) * cos(lat) * cos(lon);
    double y = (N + alt) * cos(lat) * sin(lon);
    double z = (N * (1 - e2) + alt) * sin(lat);
    ecef[0] = x;
    ecef[1] = y;
    ecef[2] = z;
    ROS_INFO("ECEF: x = %f, y = %f, z = %f", x, y, z);
}

void computeENU(double enu[], double ecef[], double LLA_ref[])
{
    // Convert Cartesian ECEF to ENU
    double lat_ref = LLA_ref[0]; // reference latitude
    double lon_ref = LLA_ref[1]; // reference longitude
    double alt_ref = LLA_ref[2]; // reference altitude

    double ecef_r[3];
    computeECEF(ecef_r, lat_ref, lon_ref, alt_ref);

    double x = ecef[0] - ecef_r[0];
    double y = ecef[1] - ecef_r[1];
    double z = ecef[2] - ecef_r[2];

    double east = - sin(lon_ref) * x + cos(lon_ref) * y;
    double north = - sin(lat_ref) * cos(lon_ref) * x - sin(lat_ref) * sin(lon_ref) * y + cos(lat_ref) * z;
    // double up = cos(lat_ref) * cos(lon_ref) * x + cos(lat_ref) * sin(lon_ref) * y + sin(lat_ref) * z;
    double up = 0.0; // keep everything in 2D
    enu[0] = east;
    enu[1] = north;
    enu[2] = up;
    ROS_INFO("ENU: east = %f, north = %f, up = %f", east, north, up);
}

void computeQuaternion(double quaternion[], double roll, double pitch, double yaw)
{
    double cy = cos(yaw / 2);
    double sy = sin(yaw / 2);
    double cp = cos(pitch / 2);
    double sp = sin(pitch / 2);
    double cr = cos(roll / 2);
    double sr = sin(roll / 2);

    double x = cy * cp * sr - sy * sp * cr;
    double y = sy * cp * sr + cy * sp * cr;
    double z = sy * cp * cr - cy * sp * sr;
    double w = cy * cp * cr + sy * sp * sr;

    quaternion[0] = x;
    quaternion[1] = y;
    quaternion[2] = z;
    quaternion[3] = w;
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
    double translation[3];
    double pitch;
    double yaw;
    double roll;
    double orientation_quaternion[4];
    double lat_r;
    double lon_r;
    double alt_r;
    double lla_rad[3];
    double ecef_r[3];

    // get reference parameters from launch file
    nh.param("/lat_r", lat_r, 1.0);
    nh.param("/lon_r", lon_r, 1.0);
    nh.param("/alt_r", alt_r, 1.0);
    lat_r = lat_r * (M_PI/180);
    lon_r = lon_r * (M_PI/180);
    alt_r = alt_r;
    double LLA_ref[] = {lat_r, lon_r, alt_r};

    // loop until ROS is shutdown
    while (ros::ok())
    {
        // create an Odometry message
        nav_msgs::Odometry odom_msg;
        
        lla_rad[0] = lat * (M_PI/180);
        lla_rad[1] = lon * (M_PI/180);
        lla_rad[2] = alt;

        // Convert Cartesian LLA to ENU
        computeECEF(ecef, lla_rad[0], lla_rad[1], lla_rad[2]);
        computeENU(enu, ecef, LLA_ref);

        // Create and populate the Odometry message
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
        
        pitch = atan(translation[2] / translation[1]) * (180/M_PI);
        yaw = atan(translation[0] / translation[1]) * (180/M_PI);
        roll = atan(translation[0] / translation[2]) * (180/M_PI);

        computeQuaternion(orientation_quaternion, roll, pitch, yaw); 

        odom_msg.pose.pose.orientation.x = orientation_quaternion[0];
        odom_msg.pose.pose.orientation.y = orientation_quaternion[1];
        odom_msg.pose.pose.orientation.z = orientation_quaternion[2];
        odom_msg.pose.pose.orientation.w = orientation_quaternion[3];

        computeECEF(ecef_r, lat_r, lon_r, alt_r);

        odom_msg.twist.twist.linear.x = ecef_r[0];
        odom_msg.twist.twist.linear.y = ecef_r[1];
        odom_msg.twist.twist.linear.z = ecef_r[2];

        pub.publish(odom_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}