#include "rrt_planning.h"
#include <tf/transform_datatypes.h>
#include <angles/angles.h>


int main(int argc, char **argv){
	/*
	initialize tree
	get costmap
	get current position
	get goal
	while(pathNotFound)
		while(true)
			generate Point in certain area	 
			if(Point close enough to other point in tree)
				break
		find best command to get to point
		integrate command and save command and endpoint to tree
		if(close enough to goal)
			pathNotFound = false
	send commands to controller
	wait if needs to replan
	*/
	ros::init(argc, argv, "rrt_planning") ;
  ros::NodeHandle nh;
  tree rrt(nh);
  ros::Rate spinRate(10);
  ros::Rate r(100000);
  
  srand(time(NULL));
  rrt.clearMarker();
  
	while(ros::ok()){
		ros::spinOnce();
		
		if(rrt.initialPoseFound && rrt.goalFound && rrt.mapFound){
		  printf("starting rrt...\n");
		  
			while(rrt.pointsInTree < rrt.maxPoints-1 && ros::ok()){
				rrt.generatePoint();
				//r.sleep();
				usleep(5000);
				
				if(rrt.pathFound){
					printf("path found\n");
					break;
				}
			}
		}
		
    if(rrt.pathFound == true){
      //TODO loop sending commands to robot with certain rate-> ros::Rate
      //TODO run multiple times and save cmd lists, before sending commands. choose cheapest cmd list and then send
      printf("sending commands...\n");
      rrt.pathCmd();
      break;
    }
    
	  if(rrt.pointsInTree >= rrt.maxPoints-1){
	    printf("maxPoints exceeded\n");
	    break;
	  }
		spinRate.sleep();
		
	}
  return 0;
}

//---Callback functions---
//------------------------

void tree::mapCallback(const nav_msgs::OccupancyGrid& msg){
	map = msg;
	
  mapFound = true;
  printf("map detected\n");
}

void tree::currentPoseCallback(const nav_msgs::Odometry& msg){
  if(initialPoseFound == false){
		treePoints[1].id = 1;
		treePoints[1].pose = msg.pose.pose;
		treePoints[1].cost = 0.0f;
		
		treePoints[1].cmd.linear.x = 0.0f;
		treePoints[1].cmd.linear.y = 0.0f;
		treePoints[1].cmd.linear.z = 0.0f;
		treePoints[1].cmd.angular.x = 0.0f;
		treePoints[1].cmd.angular.y = 0.0f;
		treePoints[1].cmd.angular.z = 0.0f;
    
    initialPoseFound = true;
    printf("Initial Position is set to x=%f and y=%f\n", treePoints[1].pose.position.x, treePoints[1].pose.position.y);
    markerPoint(treePoints[1].pose, 1);
  }
}

void tree::goalCallback(const geometry_msgs::Point& msg){
	if(goalFound == false){
		treePoints[0].id = 0;
		treePoints[0].pose.position = msg;
		treePoints[0].cost = 0.0f;
		
		treePoints[0].cmd.linear.x = 0.0f;
		treePoints[0].cmd.linear.y = 0.0f;
		treePoints[0].cmd.linear.z = 0.0f;
		treePoints[0].cmd.angular.x = 0.0f;
		treePoints[0].cmd.angular.y = 0.0f;
		treePoints[0].cmd.angular.z = 0.0f;
		
		goalFound = true;
		printf("Goal set to x=%f and y=%f\n", treePoints[0].pose.position.x, treePoints[0].pose.position.y);
    markerPoint(treePoints[0].pose, 0);
	}
}

//-------Functions--------
//------------------------

float tree::distance(geometry_msgs::Point point1, geometry_msgs::Point point2){
	float distance = sqrt(pow(point1.x-point2.x,2)+pow(point1.y-point2.y,2));
	return distance;
}

void tree::generatePoint(){
  printf("generating Point...\n");
	geometry_msgs::Point goal = treePoints[0].pose.position;
	geometry_msgs:: Point origin = treePoints[1].pose.position;
	bool closeEnough = false;
	bool connectionSuccess = false;
	int cmdSuccess;
	
	geometry_msgs::Point sampledPoint;
	
	float tempDistance;
	int closestId;
	float closestDistance = 10000.0f;
	
	//data & calculations for sampling bias
	float distanceX = goal.x - origin.x;
	float distanceY = goal.y - origin.y;
	if(abs(distanceX) < 5)
	  distanceX = 5;
	if(abs(distanceY) < 5)
	  distanceY = 5;
	
	int invX = 1, invY = 1; 
	if(distanceX < 0)
	  invX = -1;
	if(distanceY < 0)
	  invY = -1;
	
	
	
	while(!connectionSuccess){
		closeEnough = false;
		
		//sampling Point and looking if it's close enough
		while(!closeEnough){
		  sampledPoint = origin;
			sampledPoint.x += (rand() % (int)(2*distanceX*1000)) / 1000.0 * invX - distanceX / 2;
			sampledPoint.y += (rand() % (int)(2*distanceY*1000)) / 1000.0 * invY - distanceY / 2;
			
		
			for(int n = 1; n<pointsInTree; n++){
				tempDistance = distance(treePoints[n].pose.position, sampledPoint);
			
				if(tempDistance < closestDistance){
					closestDistance = distance(treePoints[n].pose.position, sampledPoint);
					closestId = n;
			
					if(closestDistance<maxPointDistance){
						closeEnough = true;
					}
				}
			}
		}
		//try to steer towards it
		cmdSuccess = generateCommand(sampledPoint, closestId);
		
		if(cmdSuccess==0){
		  printf("closest Point: #%i\n", closestId);
			connectionSuccess = true;
			treePoints[pointsInTree].id = pointsInTree;
			treePoints[pointsInTree].parentId = treePoints[closestId].id;
			
			markerPoint(treePoints[pointsInTree].pose, pointsInTree);
      markerList(pointsInTree);
		  
		  if(distance(treePoints[pointsInTree].pose.position, treePoints[0].pose.position)<2){
		    pathFound = true;
		    treePoints[0].parentId = pointsInTree;
		    printf("%i\n", treePoints[0].parentId);
		    createPath();
		  }
			pointsInTree++;
		}
	}
}

int tree::generateCommand(geometry_msgs::Point goal, int startId){
	geometry_msgs::Twist tempCmd;
	geometry_msgs::Pose tempPose;
	float tempCost;
	
	float closestDistance = 1000.0f;
	float tempDistance = 0.0f;
	int success = -1;
	
	int samplingNumber = 10;
	float speed = 1.0f;
	float angMax = 1.0f;
	
	tempCmd.linear.x = speed;
	tempCmd.linear.y = 0.0f;
	tempCmd.linear.z = 0.0f;
	tempCmd.angular.x = 0.0f;
	tempCmd.angular.y = 0.0f;
	
	
	
	for(int n = 0; n<samplingNumber; n++){
		tempCmd.angular.z = (double)(rand() % 1001 -500)/1001*angMax;
		
		if(cmdIntegration(tempCmd.linear.x, treePoints[startId].pose, tempCmd, &tempPose, &tempCost)==0){
			tempDistance = distance(goal, tempPose.position);
		
			if(tempDistance < closestDistance){
				closestDistance = tempDistance;
				
				treePoints[pointsInTree].pose = tempPose;
				treePoints[pointsInTree].cmd = tempCmd;
				treePoints[pointsInTree].cost = tempCost;
				
				success = 0;
			}
		}
	}	
	return success;
}

int tree::cmdIntegration(float speed, geometry_msgs::Pose start, geometry_msgs::Twist cmd, geometry_msgs::Pose* endPtr, float* costPtr){
		float dx = 0.0f;
		float dy = 0.0f;
		geometry_msgs::Pose tempPose;
		float T = maxPointDistance/speed;
		int subSteps = interpolationSteps;
				
		float angz = cmd.angular.z;
		float d =	speed*T/subSteps;
	  tempPose.orientation.z = tf::getYaw(start.orientation);
    tempPose.position.x = start.position.x - d*sin(tempPose.orientation.z);
	  tempPose.position.y = start.position.y + d*cos(tempPose.orientation.z);
	   
	  for(int i = 1; i<subSteps; i++){
	    tempPose.orientation.z += angz*T/(subSteps-1);
	  
	    dx = d*cos(tempPose.orientation.z);
	    tempPose.position.x += dx;

 	    dy = d*sin(tempPose.orientation.z);
	    tempPose.position.y += dy;
	    
	    if(collisionCheck(tempPose.position))
	    	return -1;
	  }
	  tf::Quaternion q_rot, q_new, q_orig;
		tf::quaternionMsgToTF(start.orientation, q_orig);
	  q_rot = tf::createQuaternionFromRPY(0,0,cmd.angular.z*T);
	  q_new = q_rot*q_orig;
	  q_new.normalize();
		tf::quaternionTFToMsg(q_new, tempPose.orientation);
	  
	  *endPtr = tempPose;
	  *costPtr = speed*T;
	  return 0;
}

bool tree::collisionCheck(geometry_msgs::Point point){
  float x = point.x;
  float y = point.y;
  bool collision = false;
  
  //convert collision to costmap coordinates
  x -= map.info.origin.position.x;
  y -= map.info.origin.position.y;
  
  //convert coordinates to cells
  int cellx = x/map.info.resolution;
  int celly = y/map.info.resolution;
  
  //check value at point
  float value = map.data[cellx + celly*map.info.width];
  //printf("value: %f\n", value);
  
  if(value > 20.0){
    return true;
    //printf("collision detected\n");
  }
  return collision;
}

geometry_msgs::Point tree::cellToCoord(int cell){
  geometry_msgs::Point coord;
  int cellX = cell % map.info.width + 1;
  int cellY = cell / map.info.height - 1;
  
  coord.x = cellX * map.info.resolution + map.info.origin.position.x;
  coord.y = cellY * map.info.resolution + map.info.origin.position.y;
  
  return coord;
}

int tree::coordToCell(geometry_msgs::Point coord){
  float cell = 0;

  cell += (coord.x - map.info.origin.position.x) / map.info.resolution;
  cell += (coord.y - map.info.origin.position.y) / map.info.resolution * map.info.width;
  
  return (int)(cell);
}

void tree::pathCmd(){
	ros::Rate cmdRate(1);
  treePose pathCmd = treePoints[treePoints[0].parentId];
  int tempId = 1;
   
	while(pathCmd.id != 0){
			
		while(pathCmd.parentId != tempId){
			pathCmd = treePoints[pathCmd.parentId];
		}
				
		tempId = pathCmd.id;
		cmdRate.sleep();
		pubCmd.publish(treePoints[tempId].cmd);
	}
			
	pubCmd.publish(treePoints[0].cmd);
}
