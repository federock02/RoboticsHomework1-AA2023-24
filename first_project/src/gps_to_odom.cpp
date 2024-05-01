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

double computeN(double latitude, double e2, double a)
{
    return (a / (sqrt(1 - (e2 * sin(latitude) * sin(latitude)))));// prime vertical radius of curvature (distance from the surface to the Z-axis along the ellipsoid normal)
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

    // variables declaration
    double lat_r;
    double lon_r;
    double alt_r;
    double old_enu[3] = {0, 0, 0};
    double ecef[3];
    double ecef_r[3];
    double enu[3];
    double delta_ECEF[3];
    double translation[3];
    double pitch;
    double yaw;
    double old_yaw = 0.0;
    double roll;
    double orientation_quaternion[4] = {0, 0, 0, 0};
    double lla_ref_rad[3];
    double lla_rad[3];
    double N;
    double shift_angle = 130.0 * (M_PI/180);

    // parameters for the WGS84 ellipsoid
    double a = 6378137.0; // semi-major axis of the Earth
    double b = 6356752.3; // semi-minor axis of the Earth
    double e2 = 1 - ((b * b) / (a * a)); // eccentricity of the Earth

    // get reference parameters from launch file
    nh.getParam("/gps_to_odom/lat_r", lat_r);
    nh.getParam("/gps_to_odom/lon_r", lon_r);
    nh.getParam("/gps_to_odom/alt_r", alt_r);

    lla_ref_rad[0] = lat_r * (M_PI/180);
    lla_ref_rad[1] = lon_r * (M_PI/180);
    lla_ref_rad[2] = alt_r;

    // Convert Cartesian LLA to ECEF for reference point
    N = computeN(lla_ref_rad[0], e2, a);
    ecef_r[0] = (N + lla_ref_rad[2]) * cos(lla_ref_rad[0]) * cos(lla_ref_rad[1]);
    ecef_r[1] = (N + lla_ref_rad[2]) * cos(lla_ref_rad[0]) * sin(lla_ref_rad[1]);
    ecef_r[2] = (N * (1 - e2) + lla_ref_rad[2]) * sin(lla_ref_rad[0]);


    // loop until ROS is shutdown
    while (ros::ok())
    {
        // create an Odometry message
        nav_msgs::Odometry odom_msg;

        lla_rad[0] = lat * (M_PI/180);
        lla_rad[1] = lon * (M_PI/180);
        lla_rad[2] = alt;

        // Convert Cartesian LLA to ENU
        N = computeN(lla_rad[0], e2, a);
        ecef[0] = (N + lla_rad[2]) * cos(lla_rad[0]) * cos(lla_rad[1]);
        ecef[1] = (N + lla_rad[2]) * cos(lla_rad[0]) * sin(lla_rad[1]);
        ecef[2] = (N * (1 - e2) + lla_rad[2]) * sin(lla_rad[0]);

        // Convert Cartesian ECEF to ENU
        delta_ECEF[0] = ecef[0] - ecef_r[0];
        delta_ECEF[1] = ecef[1] - ecef_r[1];
        delta_ECEF[2] = ecef[2] - ecef_r[2];

        enu[0] = - sin(lla_ref_rad[1]) * delta_ECEF[0] + cos(lla_ref_rad[1]) * delta_ECEF[1];
        enu[1] = - sin(lla_ref_rad[0]) * cos(lla_ref_rad[1]) * delta_ECEF[0] - sin(lla_ref_rad[0]) * sin(lla_ref_rad[1]) * delta_ECEF[1] + cos(lla_ref_rad[0]) * delta_ECEF[2];
        enu[2] = cos(lla_ref_rad[0]) * cos(lla_ref_rad[1]) * delta_ECEF[0] + cos(lla_ref_rad[0]) * sin(lla_ref_rad[1]) * delta_ECEF[1] + sin(lla_ref_rad[0]) * delta_ECEF[2];

        // ENU value needs to be shifted by 130 degrees
        double x = cos(shift_angle) * enu[0] - sin(shift_angle) * enu[1];
        double y = sin(shift_angle) * enu[0] + cos(shift_angle) * enu[1];
        enu[0] = x;
        enu[1] = y;

        // Populate the Odometry message
        odom_msg.pose.pose.position.x = enu[0];
        odom_msg.pose.pose.position.y = enu[1];
        // odom_msg.pose.pose.position.z = enu[2];
        odom_msg.pose.pose.position.z = 0;

        // Calculate the orientation of the robot in the ENU frame, using consecutive poses estimation
        translation[0] = enu[0] - old_enu[0];
        translation[1] = enu[1] - old_enu[1];
        translation[2] = enu[2] - old_enu[2];
        old_enu[0] = enu[0];
        old_enu[1] = enu[1];
        old_enu[2] = enu[2];
        
        // roll = atan(translation[0] / translation[2]);
        roll = 0.0;
        // pitch = atan(translation[2] / translation[1]);
        pitch = 0.0;
        if (translation[0] == 0)
        {
            yaw = old_yaw;
        }
        else
        {
            yaw = atan2(translation[1], translation[0]);
            old_yaw = yaw;
        }

        // Convert the Euler angles to a quaternion
        double cr = cos(roll / 2);
        double sr = sin(roll / 2);
        double cp = cos(pitch / 2);
        double sp = sin(pitch / 2);
        double cy = cos(yaw / 2);
        double sy = sin(yaw / 2);
        
        // Convert the Euler angles to a quaternion
        orientation_quaternion[0] = sr * cp * cy - cr * sp * sy;
        orientation_quaternion[1] = cr * sp * cy + sr * cp * sy;
        orientation_quaternion[2] = cr * cp * sy - sr * sp * cy;
        orientation_quaternion[3] = cr * cp * cy + sr * sp * sy;

        // Populate the Odometry message
        // odom_msg.pose.pose.orientation.x = orientation_quaternion[0];
        odom_msg.pose.pose.orientation.x = 0;
        // odom_msg.pose.pose.orientation.y = orientation_quaternion[1];
        odom_msg.pose.pose.orientation.y = 0;
        odom_msg.pose.pose.orientation.z = orientation_quaternion[2];
        odom_msg.pose.pose.orientation.w = orientation_quaternion[3];

        // Publish the Odometry message
        pub.publish(odom_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}