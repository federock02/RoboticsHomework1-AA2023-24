
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "dynamic_reconfigure/server.h"
#include "first_project/lidarConfig.h" // Importa il file di configurazione dinamica generato

#include "std_msgs/String.h"

class LidarRemapNode {
private:
    std::string frame; //frame a cui devono essere rimappati i messaggi
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Publisher remapped_pub;
    dynamic_reconfigure::Server<frame::RemapConfig> dyn_reconf_server;
    ros::Timer timer;

public:
    LidarRemapNode() {
        odom_sub = nh.subscribe("/os_cloud_node/points", 1, &LidarRemapNode::odomCallback, this);
        remapped_pub = nh.advertise<nav_msgs::Odometry>("/pointcloud_remapped", 1);
        dyn_reconf_server.setCallback(boost::bind(&LidarRemapNode::reconfigureCallback, this, _1, _2));
        frame = "wheel_odom"; // Imposta il frame predefinito
        timer = nh.createTimer(ros::Duration(1.0), &LidarRemapNode::timerCallback, this); // Timer a 1 Hz
    }
/*
Questo metodo viene chiamato ogni volta che il nodo riceve un messaggio sul topic /os_cloud_node/points.
Rimappa il frame del messaggio in base al valore attuale di frame e lo pubblica sul topic /pointcloud_remapped.
*/
    void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
        nav_msgs::Odometry remapped_msg = *msg;
        remapped_msg.header.frame_id = frame;
        remapped_pub.publish(remapped_msg);
    }

/*
Questo metodo viene chiamato ogni volta che viene modificato un parametro nel server di configurazione
dinamica. Aggiorna il valore di frame in base alla nuova configurazione.
*/
    void reconfigureCallback(frame::RemapConfig &config, uint32_t level) {
        frame = config.frame;
    }

/*
Questo metodo viene chiamato a intervalli regolari dal timer. Pubblica il valore corrente di
frame sul topic /pointcloud_remapped.
*/
    void timerCallback(const ros::TimerEvent& event) {
        remapped_pub.publish(frame);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_remap");
    LidarRemapNode lidar_remap_node;
    ros::spin();
    return 0;
}

