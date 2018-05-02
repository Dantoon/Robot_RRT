#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Int16.h"

#include <cstdlib>
#include <stdio.h>
#include <cmath>
#include <time.h>


class rrt{

  public:
    rrt(ros::NodeHandle);
    ~rrt(){}
    
    struct rrtNode{
      geometry_msgs::Twist cmd;
      geometry_msgs::Pose endPose;
      float distanceToGoal;
      float totalCost;
    };
    
    rrtNode tree[10000];
    
    unsigned int nodeCounter;
    int maxNodes;
    int nodesToGoal;
    int generateCmd();
    int closestNode();
    void createPath();
    
    float goalArea;
    bool mapGoalExist;
    bool initialPoseExist;
    bool pathFound;
    
    nav_msgs::OccupancyGrid map;
   	visualization_msgs::Marker treeMarker;
    
    float distanceX;
  	float distanceY;
  	
  	void markerPoint(geometry_msgs::Pose pose, int type);
  	void markerLine(geometry_msgs::Pose pose1, int type);  	
    void clearMarker();
    
  private:
    int markerCounter;      //delete when more elegant solution found
    
    bool validAngle;
    float badAngle;
  	float robotSize;
  	
  	float timeStep;
  	int interpolationSteps;
  	
    rrtNode closestNodes[10];
  	
    ros::Subscriber subGoal;
    ros::Subscriber subMap;
    ros::Subscriber subMapMeta;
    ros::Subscriber subPose;
    
    ros::Publisher pubPathPoints;
    ros::Publisher pubPathPointsNumber;
    ros::Publisher pubMarker;
    
    bool collisionCheck(geometry_msgs::Point point);
    
    void goalCallback(const geometry_msgs::Pose& goal_msg);
    void initialPoseCallback(const nav_msgs::Odometry& odom_msg);
    void mapCallback(const nav_msgs::OccupancyGrid& map_msg);
    void mapMetaCallback(const nav_msgs::MapMetaData& meta_msg);

};


//initializes variables of class
rrt::rrt(ros::NodeHandle nh){
  mapGoalExist = false;
  initialPoseExist = false;
  pathFound = false;
  nodeCounter = 2;
  maxNodes = 10000;
  goalArea = 0.5;
  robotSize = 1;
  srand(time(NULL));
  markerCounter = 2;
  badAngle = 100;
  validAngle = true;
  timeStep = 1;
  interpolationSteps = 30;
  
  subGoal = nh.subscribe("map_goal", 10, &rrt::goalCallback, this);
  subPose = nh.subscribe("robot_0/odom", 10, &rrt::initialPoseCallback, this);
  subMap = nh.subscribe("robot_0/robot_map/robot_map/costmap", 10, &rrt::mapCallback, this);
  
  pubPathPoints = nh.advertise<geometry_msgs::Point>("path_points", 50, true);
  pubPathPointsNumber = nh.advertise<std_msgs::Int16>("path_points_number", 50, true);
  pubMarker = nh.advertise<visualization_msgs::Marker>("treepoints",50,true);
}

//~~~Callback Functions~~~
//~~~~~~~~~~~~~~~~~~~~~~~~

void rrt::mapCallback(const nav_msgs::OccupancyGrid& msg){

  map = msg;
  printf("map detected\n");
}

void rrt::goalCallback(const geometry_msgs::Pose& msg){
  tree[0].endPose = msg;
  
  tree[0].cmd.linear.x = 0;
  tree[0].cmd.linear.y = 0;
  tree[0].cmd.linear.z = 0;
  tree[0].cmd.angular.x = 0;
  tree[0].cmd.angular.y = 0;
  tree[0].cmd.angular.z = 0;
  
  
  printf("Map Goal is set to x=%f and y=%f\n", tree[0].endPose.position.x, tree[0].endPose.position.y);
  mapGoalExist = true;
  markerPoint(tree[0].endPose,0);
}

void rrt::initialPoseCallback(const nav_msgs::Odometry& msg){
  if(initialPoseExist == false){
    tree[1].endPose = msg.pose.pose;
    
    initialPoseExist = true;
    printf("Initial Position is set to x=%f and y=%f\n", tree[1].endPose.position.x, tree[1].endPose.position.y);
  }
  markerPoint(tree[1].endPose,1);
}


//~~~Functions~~~
//~~~~~~~~~~~~~~~

int rrt::generateCmd(){
	 printf("~~~generating cmd #%i~~~\n", nodeCounter);
	//generate N command velocities which run for time T
	int N = 50;
	float angz = 0;
	float angMax =  2;
	float speed = 1;
	int success = -1;
	
	int subSteps = interpolationSteps;    //substeps for numerical integration
	float T = timeStep;                   //length of command in seconds
	float d = speed*T/subSteps;           //distance travelled per sub step
	
	geometry_msgs::Pose tempPose;
	
	float closestDistance = 10000;
	float tempDistance = 10000;
	float cost;

	for(int counter=0; counter<N; counter++){
	
		angz = (double)(rand() % 1001 -500)/1001*angMax;
		
		while(validAngle==false && abs(badAngle-angz)<0.2*angMax){
		  angz = (double)(rand() % 1001 -500)/1001*angMax;
		}
		
		//d = (double)(rand() % 1001)/1001*d;
	
	//integrate path to see determine path position after time t and distance travelled
	  float dx = 0;
	  float dy = 0;
	  int i;                      //counter for for-loops
	  
	  tempPose.orientation.z = tree[nodeCounter-1].endPose.orientation.z;
    tempPose.position.x = tree[nodeCounter-1].endPose.position.x -d*sin(tempPose.orientation.z);
	  tempPose.position.y = tree[nodeCounter-1].endPose.position.y +d*cos(tempPose.orientation.z);
	  	  
	  for(i = 1; i<subSteps; i++){
	  
	    //markerLine(tempPose, 1);
	    tempPose.orientation.z += angz*T/(subSteps-1);
	  
	    dx = -d*sin(tempPose.orientation.z);
	    tempPose.position.x += dx;

 	    dy = d*cos(tempPose.orientation.z);
	    tempPose.position.y += dy;
	   
	    if(collisionCheck(tempPose.position)==true){
	      //printf("collision in substeps, while creating path...\n");
	      break;
	    }
	  }	  
	  
	  
	//choose path closest to goal
		tempDistance = sqrt(pow(tree[0].endPose.position.x - tempPose.position.x,2)+pow(tree[0].endPose.position.y - tempPose.position.y,2));
	
		if(tempDistance < closestDistance && collisionCheck(tempPose.position)==false){
		
			closestDistance = tempDistance;
			tree[nodeCounter].cmd.linear.x = d*subSteps/T;
			tree[nodeCounter].cmd.angular.z = angz;	
			tree[nodeCounter].distanceToGoal = tempDistance;
			tree[nodeCounter].endPose = tempPose;
			success = 0;
			
			//add visualization
			markerPoint(tempPose, 2);
			//markerLine(tempPose, nodeCounter);
			//printf("angle: %f\n",angz);
			//printf("orientation: %f\n", tree[nodeCounter].endPose.orientation.z);			
		}
	}
	
	//check distance to goal
	float distance2Goal = sqrt(pow(tree[0].endPose.position.x-tree[nodeCounter].endPose.position.x,2) + pow(tree[0].endPose.position.y-tree[nodeCounter].endPose.position.y,2));
	
	if(distance2Goal < goalArea && success == 0){
	  pathFound = true;
	  nodesToGoal = nodeCounter;
	}
	
	if(success == 0){
		validAngle = true;		
	}
	
	if(success == -1){
	  validAngle = false;
	  badAngle = tree[nodeCounter-2].cmd.angular.z;
	}
	
	return success;
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
    collision = true;
    //printf("collision detected\n");
  }
  return collision;
}



//~~~Visualization~~~
//~~~~~~~~~~~~~~~~~~~

void rrt::markerPoint(geometry_msgs::Pose pose, int type){
  //type: 0-goal, 1-start, 2-generic, 3-mini
  
	visualization_msgs::Marker nodeMarker;
	nodeMarker.type = visualization_msgs::Marker::SPHERE;
	
	nodeMarker.header.frame_id = "map";
	nodeMarker.ns = "treepoint";
	nodeMarker.id = nodeCounter;
	
	if(type < 2){
	  nodeMarker.id = type;
	}
	
	nodeMarker.action = visualization_msgs::Marker::ADD;
	
	nodeMarker.color.g = 1.0f;
	nodeMarker.color.a = 1.0f;
	
	nodeMarker.scale.x = 0.4f;
	nodeMarker.scale.y = 0.4f;
	nodeMarker.scale.z = 0.1f;
	
	nodeMarker.pose = pose;
	nodeMarker.pose.position.z = 0.01f;
	nodeMarker.pose.orientation.w = 1.0f;
	
	if(type==0){
		nodeMarker.color.r = 1.0f;
		nodeMarker.color.g = 0.0f;
	}
	if(type==1){
		nodeMarker.color.b = 1.0f;
		nodeMarker.color.g = 0.0f;
	}

	pubMarker.publish(nodeMarker);	
}

void rrt::markerLine(geometry_msgs::Pose pose, int type){
  //type: 0-final path, 1-generic 
  
	treeMarker.type = visualization_msgs::Marker::LINE_STRIP;
	treeMarker.action = visualization_msgs::Marker::ADD;

	treeMarker.header.frame_id = "map";
	treeMarker.ns = "tree";
	treeMarker.id = 1;

	treeMarker.color.b = 1.0f;
	treeMarker.color.a = 1.0f;

	treeMarker.scale.x = 0.1f;
	treeMarker.pose.orientation.w = 1.0f;
	pose.position.z = 0.1f;

	treeMarker.points.push_back(pose.position);
	

	pubMarker.publish(treeMarker);
}

void rrt::createPath(){
  
  for(int n = 2; n < nodesToGoal; n++){
		float dx = 0;
		float dy = 0;//TODO declare tempPose
		geometry_msgs::Pose tempPose;
		float T = timeStep;
		int subSteps = interpolationSteps;
		
		float angz = tree[n].cmd.angular.z;
		float d =	tree[n].cmd.linear.x*T/subSteps;
	  tempPose.orientation.z = tree[n-1].endPose.orientation.z;
    tempPose.position.x = tree[n-1].endPose.position.x -d*sin(tempPose.orientation.z);
	  tempPose.position.y = tree[n-1].endPose.position.y +d*cos(tempPose.orientation.z);  
	  for(int i = 1; i<subSteps; i++){
	    markerLine(tempPose, 1);
	    tempPose.orientation.z += angz*T/(subSteps-1);
	  
	    dx = -d*sin(tempPose.orientation.z);
	    tempPose.position.x += dx;

 	    dy = d*cos(tempPose.orientation.z);
	    tempPose.position.y += dy;
	  }	
	}
}

void rrt::clearMarker(){

	visualization_msgs::Marker deleteAll;
	deleteAll.action = 3;
	pubMarker.publish(deleteAll);

}















