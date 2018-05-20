#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"

#include <stdio.h>

void mapCallback(const nav_msgs::OccupancyGrid& map_msg);

int main(int argc, char **argv){
	ros::init(argc, argv, "collision_test") ;
  ros::NodeHandle nh;
  ros::Subscriber subMap;
  
  subMap = nh.subscribe("robot_0/robot_map/robot_map/costmap", 10, &mapCallback);
	ros::spin();

	return 0;
}  
    
void mapCallback(const nav_msgs::OccupancyGrid& msg){
	printf("checking for collision...	");
	
	for(int k=0; k<msg.info.width*msg.info.height; k++){
		if(msg.data[k]>20){
			printf("collision at %i\n", k);
		}
	}
}
