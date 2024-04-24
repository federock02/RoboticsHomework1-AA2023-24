#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "dynamic_reconfigure/server.h"
#include "first_project/lidarConfig.h" // Import the generated dynamic configuration file

class LidarRemapNode {
private:
    static std::string frame; // Frame to which messages should be remapped
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub; // Subscriber for PointCloud2 messages
    ros::Publisher remapped_pub; // Publisher for PointCloud2 messages
    dynamic_reconfigure::Server<first_project::lidarConfig> dyn_reconf_server;
    ros::Timer timer;

public:
    LidarRemapNode() {
        cloud_sub = nh.subscribe("/os_cloud_node/points", 1, &LidarRemapNode::cloudCallback, this);
        remapped_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1);
        dyn_reconf_server.setCallback(boost::bind(&LidarRemapNode::reconfigureCallback, this, _1, _2));
        frame = "wheel_odom"; // Set the default frame
        timer = nh.createTimer(ros::Duration(1.0), &LidarRemapNode::timerCallback, this); // 1 Hz timer
    }

    // This method is called whenever the node receives a message on the /os_cloud_node/points topic.
    // It remaps the frame of the message based on the current value of frame and publishes it on the /pointcloud_remapped topic.
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        sensor_msgs::PointCloud2 remapped_msg = *msg;
        remapped_msg.header.frame_id = frame;
        remapped_pub.publish(remapped_msg);
    }

    // This method is called whenever a parameter is modified in the dynamic reconfigure server.
    // It updates the value of frame based on the new configuration.
    void reconfigureCallback(first_project::lidarConfig &config, uint32_t level) {
        frame = config.frame;
    }

    // This method is called at regular intervals by the timer.
    // It publishes the current value of frame on the /pointcloud_remapped topic.
    void timerCallback(const ros::TimerEvent& event) {
        sensor_msgs::PointCloud2 frame_msg;
        frame_msg.header.frame_id = frame;
        remapped_pub.publish(frame_msg);
    }
};

// Definition of the static member variable
std::string LidarRemapNode::frame;

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_remap");
    LidarRemapNode lidar_remap_node;
    ros::spin();
    return 0;
}
