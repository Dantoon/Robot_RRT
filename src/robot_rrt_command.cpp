#include <ros/ros.h>

#include "robot_rrt_command.h"

int main(int argc, char **argv){
  
  ros::init(argc, argv, "robot_rrt_planning") ;
  ros::NodeHandle nh;
  rrt robot_rrt(nh);
  robot_rrt.clearMarker();
  
  ros::Rate r(10);
  ros::Rate s(50);
  ros::spinOnce();
  
  int success;
  
  
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
				        
        if(robot_rrt.pathFound){
          printf("~~~creating path~~~\n");
          break;
        }
        //usleep(100);
        s.sleep();
      }
    }
    if(robot_rrt.pathFound == true){
      break;
    }
    
    r.sleep();	//does spinning have a performance / stability impact?-> is sleep necessary?
  }
  
}
