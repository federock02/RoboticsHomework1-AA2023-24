#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "math.h"

double lat = 0.0;
double lon = 0.0;
double alt = 0.0;

// Extract the latitude, longitude, and altitude values from the NavSatFix message from the /fix topic
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    lat = msg->latitude;
    lon = msg->longitude;
    alt = msg->altitude;
}

double computeN(double lat)
{
    double a = 6378137.0; // semi-major axis of the Earth
    double b = 6356752.3; // semi-minor axis of the Earth
    double e = sqrt(1 - (b * b) / (a * a)); // eccentricity of the Earth
    double N = a / sqrt(1 - (e * e) * (sin(lat) * sin(lat))); // prime vertical radius of curvature (distance from the surface to the Z-axis along the ellipsoid normal)
    return N;
}

double[] computeECEF(double lat, double lon, double alt)
{
    // Convert latitude, longitude, and altitude to Cartesian ECEF
    double x = (computeN(lat) + alt) * cos(lat) * cos(lon);
    double y = (computeN(lat) + alt) * cos(lat) * sin(lon);
    double z = (computeN(lat) * (1 - e * e) + alt) * sin(lat);

    double[] ecef = {x, y, z};
    return ecef;
}

double[] computeENU(double[] ecef, double[] LLA_ref)
{
    // Convert Cartesian ECEF to ENU
    double lat_r = LLA_ref[0]; // reference latitude
    double lon_r = LLA_ref[1]; // reference longitude
    double alt_r = LLA_ref[2]; // reference altitude

    double[] ecef_r = computeECEF(lat_r, lon_r, alt_r);

    double x = ecef[0] - ecef_r[0];
    double y = ecef[1] - ecef_r[1];
    double z = ecef[2] - ecef_r[2];

    double east = -sin(lon_r) * x + cos(lon_r) * y;
    double north = -cos(lon_r) * sin(lat_r) * x - sin(lat_r) * sin(lon_r) * y + cos(lat_r) * z;
    double up = cos(lat_r) * cos(lon_r) * x + cos(lat_r) * sin(lon_r) * y + sin(lat_r) * z;

    double[] enu = {east, north, up};
    return enu;
}

double[] computeQuaternion(double roll, double pitch, double yaw)
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

    double[] quaternion = {x, y, z, w};
    return quaternion;
}

int main(int argc, char const *argv[])
{
    ros::init(argc, argv, "gps_to_odom");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 1);
    
    # subscribe to the /fix topic from the robotics.bag file
    # and extract the latitude, longitude, and altitude values
    # from the NavSatFix message
    ros::Subscriber sub = nh.subscribe("/fix", 1, gpsCallback);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        // 
        nav_msgs::Odometry odom_msg;

        double[] old_ENU = {0.0, 0.0, 0.0};

        double lat_r; // reference latitude
        double lon_r; // reference longitude
        double alt_r; // reference altitude
        // get reference parameters from launch file
        nh.getParam("/first_project/lat_r", lat_r);
        nh.getParam("/first_project/lon_r", lon_r);
        nh.getParam("/first_project/alt_r", alt_r);
        double[] LLA_ref = {lat_r, lon_r, alt_r};

        // Convert Cartesian LLA to ENU
        double[] ENU = computeENU(computeECEF(lat, lon, alt), LLA_ref);

        // Create and populate the Odometry message
        odom_msg.pose.pose.position.x = ENU[0];
        odom_msg.pose.pose.position.y = ENU[1];
        odom_msg.pose.pose.position.z = ENU[2];

        // Calculate the orientation of the robot in the ENU frame, using consecutive poses estimation
        double[] translation = {ENU[0] - old_ENU[0], ENU[1] - old_ENU[1], ENU[2] - old_ENU[2]};
        double pitch = atan2(translation[2], sqrt(translation[0] * translation[0] + translation[1] * translation[1]));
        double yaw = atan2(translation[1], translation[0]);
        double roll = 0.0;

        double[] orientation_quaternion = computeQuaternion(roll, pitch, yaw); 


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