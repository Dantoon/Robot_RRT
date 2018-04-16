#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include "geometry_msgs/Point.h"

class controller{

	public:
		controller(ros::NodeHandle);
    ~controller(){}
	
	private:
		int arrCounter;
		int pointsNumber;
		geometry_msgs::Point wayPoints[];
		
		ros::Subscriber subPointsNumber;
		ros::Subscriber subPoints;
		ros::Publisher pubCmdVel;
		
		void subPointsNumberCallback(const std_msgs::Int16 number_msg);
		void subPointsCallback(const geometry_msgs::Point& points_msg);
};

controller::controller(ros::Nodehandle nh){
	arrCounter = 0;
	
  subPointsNummber = nh.subscribe("path_points_number", 10, &rrt::subPointsNumberCallback, this);
  subPoints = nh.subscribe("path_points", 10, &rrt::subPointsCallback, this);

  pubCmdVel = nh.advertise<geometry_msgs::Twist>("cmd_vel",50,true);
}

void controller::subPointsNumberCallback(const std_msgs::Int16 msg){
	pointsNumber = msg.data;
}

void controller subPointsCallback(const geometry_msgs::Point msg){
	geometry_msgs::Point temp[pointsNumber];
	temp[pointsNumber-arrCounter-1] = msg;
	arrCounter++;
	
	wayPoints[] = temp[];
	
	if(arrCounter == pointsNumber){
		printf("waypoint transmission finished\n");
	}
}











