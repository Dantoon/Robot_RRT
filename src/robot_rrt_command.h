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
    int generateCmd();
    int closestNode();
    
    float goalArea;
    bool mapGoalExist;
    bool initialPoseExist;
    bool pathFound;
    
    nav_msgs::OccupancyGrid map;
    
    float distanceX;
  	float distanceY;
  	
  	void markerPoint(geometry_msgs::Pose pose, int type);
  	void markerLine(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, int type);  	
    void clearMarker();
    
  private:
  	float robotSize;
  	
    rrtNode closestNodes[10];
  	
    ros::Subscriber subGoal;
    ros::Subscriber subMap;
    ros::Subscriber subMapMeta;
    ros::Subscriber subPose;
    
    ros::Publisher pubPathPoints;
    ros::Publisher pubPathPointsNumber;
    ros::Publisher pubMarker;
    
    bool collisionCheck(geometry_msgs::Point point1, geometry_msgs::Point point2);
    
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
  goalArea = 1;
  robotSize = 1;
  srand(time(NULL));
  
  
  subGoal = nh.subscribe("map_goal", 10, &rrt::goalCallback, this);
  subPose = nh.subscribe("robot_0/odom", 10, &rrt::initialPoseCallback, this);
  subMap = nh.subscribe("robot_0/robot_map/robot_map/costmap", 10, &rrt::mapCallback, this);
  
  pubPathPoints = nh.advertise<geometry_msgs::Point>("path_points", 50, true);
  pubPathPointsNumber = nh.advertise<std_msgs::Int16>("path_points_number", 50, true);
  pubMarker = nh.advertise<visualization_msgs::Marker>("treepoints",50,true);
}

void rrt::mapCallback(const nav_msgs::OccupancyGrid& msg){

  map = msg;
  printf("map detected\n");
}

void rrt::goalCallback(const geometry_msgs::Pose& msg){
  tree[0].endPose = msg;
  
  
  
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


int rrt::generateCmd(){
	 printf("~~~generating cmd #%i~~~\n", nodeCounter);
	//generate n command velocities which run for time t
	int N = 100;
	float angz;
	float angMax =  1;
	float speed = 1;
	int success = -1;
	
	geometry_msgs::Pose tempPose;
	geometry_msgs::Twist tempCmd;
	
	float closestDistance = 10000;
	float tempDistance = 10000;
	float cost;

	for(int counter=0; counter<N; counter++){
	
		angz = (double)(rand() % 101 -50)/101*angMax;
	
	//integrate path to see determine path position after time t and distance travelled
	  int subSteps = 10;
	  float T = 1;          //length of command in seconds
	  float d = T/subSteps;
	  float dx = 0;
	  float dy = 0;
	
	  int i;
	  
	  for(i = 0; i<subSteps; i++){
	   dx += d*cos(tree[nodeCounter-1].endPose.orientation.z+i*angz*T/subSteps);
	  }
    for(i = 0; i<subSteps; i++){
	   dy += d*sin(tree[nodeCounter-1].endPose.orientation.z+i*angz*T/subSteps);
	  }
    	
    //printf("dx: %f  dy: %f\n", dx,dy);	
    	 
    tempPose.position.x = tree[nodeCounter-1].endPose.position.x + dx;
	  tempPose.position.y = tree[nodeCounter-1].endPose.position.y + dy;
	  tempPose.orientation.z = tree[nodeCounter-1].endPose.orientation.z + angz*T;
	
	//choose path closest to goal
		tempDistance = sqrt(pow(tree[0].endPose.position.x - tempPose.position.x,2)+pow(tree[0].endPose.position.y - tempPose.position.y,2));
	
		if(tempDistance < closestDistance){
		
		//TODO collision Checking
			closestDistance = tempDistance;
			tree[nodeCounter].cmd.linear.x = speed;
			tree[nodeCounter].cmd.angular.z = angz;	
			tree[nodeCounter].distanceToGoal = tempDistance;
			tree[nodeCounter].endPose = tempPose;
			success = 0;
			
			//add visualization
			markerPoint(tempPose, 2);
			printf("angle: %f\n",angz);
			printf("orientation: %f\n", tree[nodeCounter].endPose.orientation.z);			
		}
	}
	
	//check distance to goal
	float distance2Goal = sqrt(pow(tree[0].endPose.position.x-tree[nodeCounter].endPose.position.x,2) + pow(tree[0].endPose.position.y-tree[nodeCounter].endPose.position.y,2));
	
	if(distance2Goal < goalArea && success == 0){
	  pathFound = true;
	}
	
	return success;
}

bool rrt::collisionCheck(geometry_msgs::Point point1, geometry_msgs::Point point2){

}


//Visualization

void rrt::markerPoint(geometry_msgs::Pose pose, int type){
  //type: 0-goal, 1-start, 2-generic  
  
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
	if(type==0){
		nodeMarker.color.r = 1.0f;
		nodeMarker.color.g = 0.0f;
	}
	if(type==1){
		nodeMarker.color.b = 1.0f;
		nodeMarker.color.g = 0.0f;
	}
	nodeMarker.color.a = 1.0f;
	
	nodeMarker.scale.x = 0.4f;
	nodeMarker.scale.y = 0.4f;
	nodeMarker.scale.z = 0.1f;
	
	nodeMarker.pose = pose;
	nodeMarker.pose.position.z = 0.01f;
	nodeMarker.pose.orientation.w = 1.0f;
	
	pubMarker.publish(nodeMarker);	
}

void rrt::markerLine(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, int type){
  //type: 0-final path, 1-generic 
  

	visualization_msgs::Marker treeMarker;
	treeMarker.type = visualization_msgs::Marker::LINE_LIST;
	treeMarker.action = visualization_msgs::Marker::ADD;

	treeMarker.header.frame_id = "map";
	treeMarker.ns = "tree";
	treeMarker.id = nodeCounter;

	treeMarker.color.b = 1.0f;
	treeMarker.color.a = 1.0f;
	
	if(type == 0){
		treeMarker.color.b = 0.0f;
		treeMarker.color.r = 1.0f;
	}

	treeMarker.scale.x = 0.1f;
	treeMarker.pose.orientation.w = 1.0f;

	treeMarker.points.push_back(pose1.position);
	treeMarker.points.push_back(pose2.position);

	pubMarker.publish(treeMarker);
}

void rrt::clearMarker(){

	visualization_msgs::Marker deleteAll;
	deleteAll.action = 3;
	pubMarker.publish(deleteAll);

}















