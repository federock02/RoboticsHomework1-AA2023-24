#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "std_msgs/String.h"

#include <tf/transform_datatypes.h> // Include the header file for TF transformations

class OdomToTFConverter {
public:
    OdomToTFConverter() {
        sub_odom = n.subscribe("input_odom", 1, &OdomToTFConverter::odomCallback, this);

        ros::NodeHandle private_nh("~");


        private_nh.getParam("root_frame", root_frame);
        private_nh.getParam("child_frame", child_frame);

        ROS_INFO("CF %s", child_frame.c_str());
        ROS_INFO("RF %s", root_frame.c_str());
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        static tf::TransformBroadcaster br;
        tf::Transform transform;

        transform.setOrigin(tf::Vector3(msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z));


        transform.setRotation(tf::Quaternion(msg->pose.pose.orientation.x,
                 msg->pose.pose.orientation.y,
                 msg->pose.pose.orientation.z,
                 msg->pose.pose.orientation.w));

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), root_frame, child_frame));
    }

private:
    ros::NodeHandle n;
    ros::Subscriber sub_odom;

    std::string root_frame, child_frame;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_to_tf");
    OdomToTFConverter converter;
    ros::spin();
    return 0;
}
