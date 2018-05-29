#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"

class controller{

	public:
		controller(ros::NodeHandle);
    ~controller(){}
    
    bool cmdOk;
    
    void cmdPub();
	
	private:
		int errCounter;
		float error[];
		float Kp, Ki, Kd;
		float pidControl(float, float);
	
		int cmdCounter;
		int pubCounter;
	  int cmdNum;
		geometry_msgs::Twist* pathCmds;
		geometry_msgs::Point closestPoint1;
		geometry_msgs::Point closestPoint2;
		
		ros::Subscriber subCmdNum;
		ros::Subscriber subCmd;
		ros::Subscriber subCancel;
		ros::Publisher pubCmdVel;
		
		void cmdNumCallback(const std_msgs::Int16&);
		void cmdCallback(const geometry_msgs::Twist&);
		void cancelCallback(const std_msgs::Int16&);
};

//----Constructors----
//--------------------

controller::controller(ros::NodeHandle nh){
	cmdCounter = 0;
	pubCounter = 0;
	cmdOk = false;
	cmdNum = 0;
	
  subCmdNum = nh.subscribe("/cmdNum", 10, &controller::cmdNumCallback, this);
  subCmd = nh.subscribe("/cmds", 10, &controller::cmdCallback, this);
  subCancel = nh.subscribe("/cancel_rrt", 10, &controller::cancelCallback, this);

  pubCmdVel = nh.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",50,true);
}







