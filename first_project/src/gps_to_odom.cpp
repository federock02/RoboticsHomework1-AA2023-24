#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"

double computeN(double lat)
{
    double a = 6378137.0; // semi-major axis of the Earth
    double b = 6356752.3; // semi-minor axis of the Earth
    double e = sqrt(1 - (b * b) / (a * a)); // eccentricity of the Earth
    double N = a / sqrt(1 - (e * e) * (sin(lat) * sin(lat))); // prime vertical radius of curvature (distance from the surface to the Z-axis along the ellipsoid normal)
    return N;
}

int main(int argc, char const *argv[])
{
    ros::init(argc, argv, "gps_to_odom");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 1);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        nav_msgs::Odometry odom_msg;
        // Convert latitude, longitude, and altitude to Cartesian ECEF
        double x = (computeN(lat) + alt) * cos(lat) * cos(lon);
        double y = (computeN(lat) + alt) * cos(lat) * sin(lon);
        double z = (computeN(lat) * (1 - e * e) + alt) * sin(lat);


        // Convert Cartesian ECEF to ENU
        double east = 0.0; // calculate east coordinate
        double north = 0.0; // calculate north coordinate
        double up = 0.0; // calculate up coordinate

        // Create and populate the Odometry message
        odom_msg.pose.pose.position.x = east;
        odom_msg.pose.pose.position.y = north;
        odom_msg.pose.pose.position.z = up;

        // Set the reference point
        odom_msg.pose.pose.orientation.x = lat_r;
        odom_msg.pose.pose.orientation.y = lon_r;
        odom_msg.pose.pose.orientation.z = alt_r;

        pub.publish(odom_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
*/