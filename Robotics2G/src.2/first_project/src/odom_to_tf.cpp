#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <string.h>
#include "ros/node_handle.h"



class tf_sub_pub{
public:
    tf_sub_pub(){
    sub = n.subscribe("input_odom", 1000, &tf_sub_pub::callback, this);

}



void callback(const nav_msgs::Odometry & msg){

  

  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, 0) );
  tf::Quaternion q;
  double z = msg.pose.pose.orientation.z;
  double w = msg.pose.pose.orientation.w;
  double theta=atan2(z,w); //+130/180*M_PI;

  
  std::string node_name = ros::this_node::getName();
  std::string child;
  std::string res;

  res=node_name+"/"+"child";


  n.getParam(res, child);

  //res=namespace_str+"/"+child;



  //ROS_WARN("res: %s", res.c_str());

  q.setRPY(0, 0, theta);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", child));
}


private:
  ros::NodeHandle n; 
  tf::TransformBroadcaster br;
  ros::Subscriber sub;
};


int main(int argc, char **argv){
 ros::init(argc, argv, "subscribe_and_publish");
 tf_sub_pub my_tf_sub_bub;

 ros::Rate loop_rate(10);

 ros::spin();
 return 0;
}

