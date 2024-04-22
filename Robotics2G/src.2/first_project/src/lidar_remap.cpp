#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
//#include <first_project/lidar_remapConfig.h>
#include <first_project/parametersConfig.h>


ros::Publisher publisher;
std::string new_frame_id; 


void dynamicReconfigureCallback(first_project::parametersConfig &config, uint32_t level) {
    // update the new_frame_id variable with the new value from dynamic reconfigure
    new_frame_id = config.frame_id;
    //ROS_INFO("New frame_id: %s", new_frame_id.c_str());
}

// Callback function for incoming point cloud messages
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // create a new point cloud message and set equal to the recieved message
    sensor_msgs::PointCloud2 new_msg = *msg;

    // Replace "new_frame_id" with the updated parameter value  
    new_msg.header.frame_id = new_frame_id; 
    new_msg.header.stamp = ros::Time::now();
    
    // Publish the modified message
    publisher.publish(new_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_remap");
    ros::NodeHandle nh;

    publisher = nh.advertise<sensor_msgs::PointCloud2>("pointcloud_remapped", 10);

    ros::Subscriber sub = nh.subscribe("os_cloud_node/points", 10, pointCloudCallback);

    dynamic_reconfigure::Server<first_project::parametersConfig> server;
    dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}
