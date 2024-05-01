#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h" // Include the header file for TF transformations
#include "tf/transform_broadcaster.h"

class OdomToTFConverter {
public:
    OdomToTFConverter() {

        // Get the parameters from the parameter server
        ros::NodeHandle private_nh("~");
        private_nh.getParam("child_frame", child_frame);
        private_nh.getParam("root_frame", root_frame);

        // Subscribe to the /input_odom topic
        sub_odom = n.subscribe("input_odom", 1, &OdomToTFConverter::odomCallback, this);
    }

    // Callback function for the /input_odom topic
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        static tf::TransformBroadcaster br;
        tf::Transform transform;

        // Set the translation and rotation values for the transform
        transform.setOrigin(tf::Vector3(msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z));

        // Set the rotation values for the transform
        transform.setRotation(tf::Quaternion(msg->pose.pose.orientation.x,
                 msg->pose.pose.orientation.y,
                 msg->pose.pose.orientation.z,
                 msg->pose.pose.orientation.w));

        // Broadcast the transform
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
