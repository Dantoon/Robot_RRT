#include <ros/ros.h>

#include "robot_rrt_planning.h"

int main(int argc, char **argv){
  
  ros::init(argc, argv, "robot_rrt_planning") ;
  ros::NodeHandle nh;
  rrt robot_rrt(nh);
  robot_rrt.clearMarker();
  
  ros::Rate r(10);
  ros::Rate s(50);
  ros::spinOnce();
  
  int connection;
  
  
  while(ros::ok()){
  
    ros::spinOnce();
    
    if(robot_rrt.initialPoseExist && robot_rrt.mapGoalExist){
      
      robot_rrt.distanceX = robot_rrt.tree[0].point.x - robot_rrt.tree[1].point.x;
      robot_rrt.distanceY = robot_rrt.tree[0].point.y - robot_rrt.tree[1].point.y;
      
      while(robot_rrt.nodeCounter < robot_rrt.maxNodes){
      	//scanf("%c");
        robot_rrt.generateCmd();
        connection = robot_rrt.closestNode();
        
        if(connection == 0 || connection == 1){
 	        robot_rrt.marker(robot_rrt.nodeCounter, connection);
        	robot_rrt.nodeCounter++;
				}				
				        
        if(robot_rrt.pathFound){
          printf("~~~creating path~~~\n");
          robot_rrt.createPath();
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

  sleep(1);
  
}
