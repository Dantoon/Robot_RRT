#include <ros/ros.h>

#include "robot_rrt_command.h"

int main(int argc, char **argv){
  
  ros::init(argc, argv, "robot_rrt_planning") ;
  ros::NodeHandle nh;
  rrt robot_rrt(nh);
  robot_rrt.clearMarker();
  
  ros::Rate r(10);
  ros::Rate s(1000);
  ros::spinOnce();
  
  int success;
  
  ros::Publisher pubCmd;
  pubCmd = nh.advertise<geometry_msgs::Twist>("robot_0/cmd_vel", 50, true);
  
  
  while(ros::ok()){
  
    ros::spinOnce();
    
    if(robot_rrt.initialPoseExist && robot_rrt.mapGoalExist){
      
      robot_rrt.distanceX = robot_rrt.tree[0].endPose.position.x - robot_rrt.tree[1].endPose.position.x;
      robot_rrt.distanceY = robot_rrt.tree[0].endPose.position.y - robot_rrt.tree[1].endPose.position.y;
      robot_rrt.tree[1].distanceToGoal = sqrt(pow(robot_rrt.distanceX,2) + pow(robot_rrt.distanceY,2));
      
      while(robot_rrt.nodeCounter < robot_rrt.maxNodes && ros::ok()){
      	//scanf("%c");
        success = robot_rrt.generateCmd();
        
        if(success==0)
          robot_rrt.nodeCounter++;
          
        if(success==-1){
          printf("failed creating cmd. tracing back...\n");
          robot_rrt.nodeCounter-=2;
          //tell generateCmd not to follow same path if it fails repeatedly		
				}        
				        
        if(robot_rrt.pathFound){
          printf("~~~creating path~~~\n");
          //robot_rrt.createPath();
          break;
        }
        //usleep(100);
        s.sleep();
      }
    }
    
    if(robot_rrt.pathFound == true){
      //TODO loop sending commands to robot with certain rate-> ros::Rate
      //TODO run multiple times and save cmd lists, before sending commands. choose cheapest cmd list and then send
      /*
      ros::Rate cmdRate(0.5);
      
      for(int n = 1; n<=robot_rrt.nodesToGoal; n++){
        pubCmd.publish(robot_rrt.tree[n].cmd);
        cmdRate.sleep();
      }
      pubCmd.publish(robot_rrt.tree[0].cmd);
      */
      break;
    }
    
    r.sleep();	//does spinning have a performance / stability impact?-> is sleep necessary?
  }
  
}
