#include <ros/ros.h>
#include "controller.h"

int main(int argc, char **argv){

	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;
	controller robot_ctrl(nh);
	
	ros::Rate pubRate(1);
	
  while(ros::ok){
    ros::spinOnce();
    while(robot_ctrl.cmdOk){
    	printf("publishing cmds...\n");
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
    printf("cmd #%i received\n", cmdCounter);
    cmdCounter++;
    
    if(cmdCounter == cmdNum){
     cmdCounter = 0;
     cmdOk = true;
     printf("path received. %i cmds in path\n", cmdNum);
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

//-----Functions------
//--------------------
float controller::pidControl(float y, float x){
	error[errCounter] = y - x;
	//I = sum(error);
	//D = (error[errCounter]-error[errCounter-1])/Ts;
	//P = error[errCounter];
	//ctrl = Kp*P + Ki*I + Kd*D;
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










