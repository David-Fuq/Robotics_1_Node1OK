#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include <math.h>
#include <string.h>

#define WGS84_A   6378137.0
#define WGS84_BB  6356752.0
#define WGS84_E_SQUARED (1-(pow(WGS84_BB, 2)/pow(WGS84_A, 2)))
#define WGS84_B   (WGS84_A*(1-WGS84_F))
#define WGS84_E   (sqrt(2*WGS84_F - WGS84_F*WGS84_F))
#define WGS84_F   (1/WGS84_IF)
#define WGS84_IF  298.257223563

//#define lat0    45.477669461666665
//#define lon0    9.22674018     
//#define alt0    169.039



// Global variables for GPS data
double heard_altitude = 0.0;
double heard_longitude = 0.0;
double heard_latitude = 0.0;

// Callback function to process NavSatFix messages
void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& fix_msg) {
    // Store received GPS data in global variables
    heard_altitude = fix_msg->altitude;
    heard_longitude = fix_msg->longitude;
    heard_latitude = fix_msg->latitude;
}


int main(int argc, char **argv){
    // Initialize ROS node
    ros::init(argc, argv, "gps_to_odom"); 
    ros::NodeHandle nh;

    // Subscribe to the "fix" topic to receive GPS data
    ros::Subscriber fix_sub = nh.subscribe("fix", 10, fixCallback);

    // Publisher for odometry messages
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 1);


    double lat0;
    double lon0;
    double alt0;


    nh.getParam("/lat0", lat0);   
    nh.getParam("/lon0", lon0);     
    nh.getParam("/alt0", alt0); 


    ros::Rate loop_rate(10);  // Loop rate: 10 Hz

    double N_phi0 = WGS84_A / sqrt(1.0 - (WGS84_E_SQUARED * pow(sin(lat0*M_PI/180), 2)));

    //Reference points in x,y,z 

    double X_ref = (N_phi0+alt0)*cos(lat0*M_PI/180)*cos(lon0*M_PI/180);
    double Y_ref = (N_phi0+alt0)*cos(lat0*M_PI/180)*sin(lon0*M_PI/180);
    double Z_ref = ((N_phi0*(1-WGS84_E_SQUARED))+alt0)*sin(lat0*M_PI/180);

    double x0 = X_ref;
    double y0 = Y_ref;
    double angle0 = 0;
    double angle_0 = 130/180*M_PI; 

    while (ros::ok()) {
        // Create a new odometry message
        nav_msgs::Odometry new_msg;
        new_msg.header.stamp = ros::Time::now();

        // Perform desired operations on latitude, longitude, and altitude

        double N_phi = WGS84_A / sqrt(1.0 - (WGS84_E_SQUARED * pow(sin(heard_latitude*M_PI/180), 2)));
        

        //ECEF

        double X_ecef = (N_phi+heard_altitude)*cos(heard_latitude*M_PI/180)*cos(heard_longitude*M_PI/180);
        double Y_ecef = (N_phi+heard_altitude)*cos(heard_latitude*M_PI/180)*sin(heard_longitude*M_PI/180);
        double Z_ecef = ((N_phi*(1-WGS84_E_SQUARED))+heard_altitude)*sin(heard_latitude*M_PI/180);

        //Reference points in x,y,z 

        double X_ref = (N_phi0+alt0)*cos(lat0*M_PI/180)*cos(lon0*M_PI/180);
        double Y_ref = (N_phi0+alt0)*cos(lat0*M_PI/180)*sin(lon0*M_PI/180);
        double Z_ref = ((N_phi0*(1-WGS84_E_SQUARED))+alt0)*sin(lat0*M_PI/180);


        //ECEF to ENU

        double dx = X_ecef - X_ref;
        double dy = Y_ecef - Y_ref;
        double dz = Z_ecef - Z_ref;

        double X_final =  -sin(lon0 * M_PI/180)*dx  + cos(lon0 * M_PI/180)*dy;
        double Y_final = -sin(lat0 * M_PI/180)*cos(lon0 * M_PI/180)*dx - sin(lat0 * M_PI/180)*sin(lon0 * M_PI/180)*dy + cos(lat0 * M_PI/180)*dz;
        double Z_final = cos(lat0 * M_PI/180)*cos(lon0 * M_PI/180)*dx + cos(lat0 * M_PI/180)*sin(lon0 * M_PI/180)*dy + sin(lat0 * M_PI/180)*dz;


        //Angle calculation
        double Y_angle = Y_final - y0;
        double X_angle = X_final - x0;
        double angle = atan2(Y_angle,X_angle)-angle_0;
        double degrees = angle * 180 / M_PI;

        /*
        if (angle == 0.0){
            angle = angle0;
        }*/


        // Set the position in the odometry message
        new_msg.pose.pose.position.x = X_final;
        new_msg.pose.pose.position.y = Y_final;
        new_msg.pose.pose.position.z = Z_final;

        new_msg.pose.pose.orientation.x = 0;
        new_msg.pose.pose.orientation.y = 0;
        new_msg.pose.pose.orientation.z = sin(angle/2);
        new_msg.pose.pose.orientation.w = cos(angle/2);
        

        // Publish the odometry message
        odom_pub.publish(new_msg);

        ros::spinOnce();  // Process callback functions

        loop_rate.sleep();  // Wait to maintain loop rate

        y0 = Y_final;
        x0 = X_final;
        angle0 = angle;
    }

    return 0;
}