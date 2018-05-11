#include "rrt_planning.h"



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
  
  srand(time(NULL));
  
	while(ros::ok()){
		ros::spinOnce();
		
		if(rrt.initialPoseFound && rrt.goalFound && rrt.mapFound){
		
			while(pointsInTree < maxPoints-1 && ros::ok()){
				generatePoint(&rrt);
				
				if(rrt.pathFound){
					printf("path found\n");
					break;
				}
			}
		}
		spinRate.sleep();
	}
  
	

	return 0;
}

//------Constructors------
//------------------------

tree::tree(ros::Nodehandle nh){
	pointsInTree = 2;
	mapFound = false;
	initialPoseFound = false;
	goalFound = false;
	pathFound = false;
	
	maxPoints = 10000;
	treePoints = new treePose[maxPoints];
	
  subMap = nh.subscribe("robot_0/robot_map/robot_map/costmap", 10, &tree::mapCallback, this);
  subPose = nh.subscribe("robot_0/robot_map/odom", 10, &tree::mapCallback, this);
  subGoal = nh.subscribe("map_goal", 10, &tree::mapCallback, this);
}

tree::treePose::treePose(){
	cmd.linear.x = 0.0f;
	cmd.linear.y = 0.0f;
	cmd.linear.z = 0.0f;
	cmd.angular.x = 0.0f;
	cmd.angular.y = 0.0f;
	cmd.angular.z = 0.0f;

	parent = NULL;
}

//---Callback functions---
//------------------------
float distance(geometry_msgs::Point point1, geometry_msgs::Point point2)


tree::mapCallback(const nav_msgs::OccupanceGrid& msg){
	map = msg;
	
  mapFound = true;
  printf("map detected\n");
}

tree::currentPoseCallback(const geometry_msgs::Pose& msg){
  if(initialPoseFound == false){
		treePoints[1].id = 1;
		treePoints[1].pose = msg;
		treePoints[1].cost = 0.0f;
    
    initialPoseFound = true;
    printf("Initial Position is set to x=%f and y=%f\n", tree[1].endPose.position.x, tree[1].endPose.position.y);
  }
}

tree::goalCallback(const geometry_msgs::Point& msg){
	if(goalFound == false){
		treePoints[0].id = 0;
		treePoints[0].pose = msg;
		treePoints[0].cost = 0.0f;
		
		goalFound == true;
		printf("Goal set to x=%f and y=%f\n", tree[0].pose.position.x, tree[0].pose.position.y);
	}
}

//-------Functions--------
//------------------------

distance(geometry_msgs::Point point1, geometry_msgs::Point point2){
	flost distance = sqrt(pow(point1.x-point2.x,2)+pow(point1.y-point2.y,2));
	return distance;
}

void tree::generatePoint(){
	geometry_msgs::Point goal = treePoints[0];
	geometry_msgs:: Point origin = treePoints[1];
	bool closeEnough = false;
	bool connectionSuccess = false;
	int cmdSuccess;
	
	geomety_msgs::Point sampledPoint
	
	int closestId
	float closestDistance = 10000.0f;
	
	while(!connectionSuccess){
		closeEnough = false;
		
		//sampling Point and looking if it's close enough
		while(!closeEnough){
			sampledPoint.x = rand() % map.info.width;
			sampledPoint.y = rand() % map.info.height;
		
			for(int n = 1; n<pointsInTree; n++){
			
				if(distance(treePoints[n].pose.position, sampledPoint)<closestDistance){
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
			connectionSuccess = true;
			tree[pointsInTree].id = pointsInTree;
			tree[pointsInTree].parent = &tree[closestId];
			markerPoint(treePoints[pointsInTree].pose, pointsInTree);
			pointsInTree++;
		}
	}
}

int tree::generateCommand(geometry_msgs::Point goal, int start){
	geometry_msgs::Twist tempCmd;
	geometry_msgs::Pose tempEndPose;
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
		
		if(cmdIntegration(temp.cmd.linear.x, treePoints[start].endPose, tempCmd, &tempEndPose, &tempCost)==0){	//TODO saves endPoint and tempCost at pointer location. includes collision checker->returns -1 when collision
			tempDistance = distance(goal, tempEndPose.position);
		
			if(tempDistance < closestDistance){
				closestDistance = tempDistance;
				
				tree[pointsInTree].pose = tempEndPose;
				tree[pointsInTree].cmd = tempCmd;
				tree[pointsInTree].cost = tempCost;
				
				success = 0;
			}
		}
	}	
	return success;
}

int rrt::cmdIntegration(float speed, geometry_msgs::Pose start, geometry_msgs::Twist cmd, geometry_msgs::Pose* endPtr, float* costPtr){
		float dx = 0.0f;
		float dy = 0.0f;
		geometry_msgs::Pose tempPose;
		float T = maxPointDistance/speed;
		int subSteps = interpolationSteps;
		
		float angz = cmd.angular.z;
		float d =	speed*T/subSteps;
	  tempPose.orientation.z = start.orientation.z;
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
	  *endPtr = tempPose;
	  *costPtr = speed*T;
	  return 0;
}

bool rrt::collisionCheck(geometry_msgs::Point point){
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
