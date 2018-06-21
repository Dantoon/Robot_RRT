#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>

#include <stdio.h>

void odomCallback(const nav_msgs::Odometry& msg){
	float yaw = tf::getYaw(msg.pose.pose.orientation);
	printf("Yaw = %f\n", yaw);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "collision_test") ;
  ros::NodeHandle nh;
  ros::Subscriber subOdom;
  
  subOdom = nh.subscribe("/robot_1/odom", 10, &odomCallback);
	ros::spin();

	return 0;
}  
