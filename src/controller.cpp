#include <ros/ros.h>
#include "controller.h"

int main(int argc, char **argv){

	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;
	controller robot_ctrl(nh);
	
	ros::Rate pubRate(1);
	
  while(ros::ok){
    ros::spinOnce();
    if(robot_ctrl.cmdOk){
      pubRate.sleep();
      robot_ctrl.cmdPub();
    }
  }
}

//-----Callbacks------
//--------------------

void controller::cmdNumCallback(const std_msgs::Int16& msg){
  cmdNum = msg.data;
  pathCmds = new geometry_msgs::Twist[cmdNum];
}

void controller::cmdCallback(const geometry_msgs::Twist& msg){
  if(cmdNum != 0){
    pathCmds[cmdCounter] = msg;
    cmdCounter++;
    
    if(cmdCounter == cmdNum){
     cmdCounter = 0;
     cmdOk = true;
     printf("path received.\n");
    }
  }
}

void controller::cancelCallback(const std_msgs::Int16& msg){
	cmdCounter = 0;
	pubCounter = 0;
	cmdOk = false;
	cmdNum = 0;
	printf("path cancelled.\n");
}

//-----Publishers-----
//--------------------

void controller::cmdPub(){
  pubCmdVel.publish(pathCmds[pubCounter]);
  pubCounter++;
  
  if(pubCounter == cmdNum){
    pubCounter = 0;
    cmdOk = false;
    printf("publishing finished.\n");
  }
}










