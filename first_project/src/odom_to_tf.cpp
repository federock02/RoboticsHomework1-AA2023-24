#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h> // Include the header file for TF transformations

class OdomToTFConverter {
public:
    OdomToTFConverter() {
        sub_odom_wheel = n.subscribe("/wheel_odom", 1000, &OdomToTFConverter::odomCallback, this);
        sub_odom_gps = n.subscribe("/gps_odom", 1000, &OdomToTFConverter::odomCallback, this);

        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("root_frame", root_frame, "world");
        private_nh.param<std::string>("child_frame", child_frame, "odom_frame");
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        static tf2_ros::TransformBroadcaster br;
        tf::Quaternion q;
        tf::Matrix3x3 mat;
        geometry_msgs::TransformStamped transformStamped;

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = root_frame;

        if (msg->header.frame_id == "/wheel_odom") {
            transformStamped.child_frame_id = "wheel_odom_frame";
        } else if (msg->header.frame_id == "/gps_odom") {
            transformStamped.child_frame_id = "gps_odom_frame";
        } else {
            ROS_ERROR("Unknown odometry source!");
            return;
        }

        transform.setOrigin(tf::Vector3(msg->pose.pose.position.x,
                                        msg->pose.pose.position.y,
                                        msg->pose.pose.position.z));
        mat.setRPY(msg->pose.pose.orientation.x,
                   msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z);
        mat.getRotation(q);
        transform.setRotation(q);

        br.sendTransform(transformStamped);
    }

private:
    ros::NodeHandle n;
    ros::Subscriber sub_odom_wheel;
    ros::Subscriber sub_odom_gps;
    std::string root_frame, child_frame;
    tf::Transform transform;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_to_tf");
    OdomToTFConverter converter;
    ros::spin();
    return 0;
}
