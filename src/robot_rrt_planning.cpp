#include <ros/ros.h>

#include "robot_rrt_planning.h"

int main(int argc, char **argv){
  
  ros::init(argc, argv, "robot_rrt_planning") ;
  ros::NodeHandle nh;
  rrt robot_rrt(nh);
  robot_rrt.clearMarker();
  
  ros::Rate r(10);
  //ros::Rate s(500);
  ros::spinOnce();
  
  
  while(ros::ok()){
  
    ros::spinOnce();
    
    if(robot_rrt.initialPoseExist && robot_rrt.mapGoalExist){
      
      while(robot_rrt.nodeCounter < robot_rrt.maxNodes){
        robot_rrt.generateNode();
        robot_rrt.closestNode();
        robot_rrt.marker(robot_rrt.nodeCounter);
        robot_rrt.nodeCounter++;
        
        if(robot_rrt.pathFound){
          printf("~~~creating path~~~\n");
          robot_rrt.createPath();
          break;
        }
        
        //s.sleep();
      }
    }
    if(robot_rrt.pathFound == true){
      break;
    }
    
    r.sleep();	//does spinning have a performance / stability impact?-> is sleep necessary?
  }

  return 0;
  
}
