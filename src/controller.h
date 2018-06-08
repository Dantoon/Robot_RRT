#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/MapMetaData.h"

class controller{

	public:
		controller(ros::NodeHandle);
    ~controller(){}
	  float Ts;
    bool cmdOk;
		int pubCounter;

    void pathStartSet(geometry_msgs::Pose*);    
    geometry_msgs::Pose pathPoseSet(geometry_msgs::Pose);
		float error(geometry_msgs::Pose);
		//float pid(float);   
    void cmdPub(float);
	  
	  struct controlValue{
	    float P, I, D;
	    float Kp, Ki, Kd;
	    float oldError;
	    float error;
	    float ctrl;
	    controlValue();
	   };
	  
	  controlValue ctrlDeviation, ctrlSpeed, ctrlAngle;
	  float pid(controlValue*);
	
	private:
    float oldError;
		float Kp, Ki, Kd;
		float P, I, D;
	
		int cmdCounter;
	  int cmdNum;
		geometry_msgs::Twist* pathCmds;
		geometry_msgs::Twist* pathCmdsCpy;
		
		nav_msgs::MapMetaData mapMeta;
		nav_msgs::Odometry currentOdom;
		
		ros::Subscriber subCmdNum;
		ros::Subscriber subCmd;
		ros::Subscriber subCancel;
		ros::Subscriber subOdom;
		ros::Subscriber subMapMeta;
		ros::Publisher pubCmdVel;
		
		void cmdNumCallback(const std_msgs::Int16&);
		void cmdCallback(const geometry_msgs::Twist&);
		void cancelCallback(const std_msgs::Int16&);
		void odomCallback(const nav_msgs::Odometry&);
		void mapMetaCallback(const nav_msgs::MapMetaData&);
};

//----Constructors----
//--------------------

controller::controller(ros::NodeHandle nh){
	cmdCounter = 0;
	pubCounter = 0;
	cmdOk = false;
	cmdNum = 0;
	
	Ts = 0.02f;
	oldError = 0.0f;
	Kp = 0.5f;
	Ki = 0.02f;
	Kd = 0.02f;
	P = 0.0f;
	I = 0.0f;
	D = 0.0f;
	
  subCmdNum = nh.subscribe("/cmdNum", 10, &controller::cmdNumCallback, this);
  subCmd = nh.subscribe("/cmds", 50, &controller::cmdCallback, this);
  subCancel = nh.subscribe("/cancel_rrt", 10, &controller::cancelCallback, this);
  subOdom = nh.subscribe("/robot_0/odom", 10, &controller::odomCallback, this);
  subMapMeta = nh.subscribe("/map_metadata", 10, &controller::mapMetaCallback, this);

  pubCmdVel = nh.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",50,true);
}

controller::controlValue::controlValue(){
  error = 0.0f;
	oldError = 0.0f;
	Kp = 0.5f;
	Ki = 0.02f;
	Kd = 0.02f;
	P = 0.0f;
	I = 0.0f;
	D = 0.0f;  
}





