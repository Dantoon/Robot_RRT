#include <ros/ros.h>

#include "controller.h"

int main(int argc, char **argv){

	ros::init(argc, argv, "controller");
	ros::Nodehandle nh;
	controller robot_ctrl(nh);
	
	ros::spin();

}
