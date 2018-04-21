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
      geometry_msgs::Point point;
      int parentID;
      float cost;
    };
    
    rrtNode tree[10000];
    
    rrtNode newNode;
    unsigned int nodeCounter;
    int maxNodes;
    void generateNode();
    int closestNode();
    void createPath();
    void marker(int markerNum);
    void clearMarker();
    
    float goalArea;
    bool mapGoalExist;
    bool initialPoseExist;
    bool pathFound;
    
    nav_msgs::OccupancyGrid map;
    
    float distanceX;
  	float distanceY;
    
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
    
    void goalCallback(const geometry_msgs::Twist& goal_msg);
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
  goalArea = 2;
  robotSize = 1;
  
  
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

void rrt::goalCallback(const geometry_msgs::Twist& msg){
  tree[0].point.x = msg.linear.x;
  tree[0].point.y = msg.linear.y;
  
  printf("Map Goal is set to x=%f and y=%f\n", tree[0].point.x, tree[0].point.y);
  mapGoalExist = true;
  marker(0);
}

void rrt::initialPoseCallback(const nav_msgs::Odometry& msg){
  if(initialPoseExist == false){
    tree[1].point.x = msg.pose.pose.position.x;
    tree[1].point.y = msg.pose.pose.position.y;
    tree[1].parentID = -1;
    tree[1].cost = 0.0;
    srand(time(NULL));
    
    initialPoseExist = true;
    printf("Initial Position is set to x=%f and y=%f\n", tree[1].point.x, tree[1].point.y);
  }
  marker(1);
}


void rrt::generateNode(){
	//TODO change algorithm to spawn nodes in area roughly in between goal and start. cannot check for in map anymore with costmap (out of bounds is 0 not -1)
  float cellX;
  float cellY;
  bool occupied = true;
  
  /*
  while(occupied){
    cellX = rand() % map.info.width;
    cellY = rand() % map.info.height;
    if(map.data[cellX*map.info.width + cellY - 1] < 30){
      occupied = false;
    }
  }
  */
  
  int spawnAreaX = 2*(distanceX)/map.info.resolution;
  int spawnAreaY = 2*(distanceY)/map.info.resolution;
  int invX;
  int invY;
  
  if (spawnAreaX >= 0)
  	invX = 1;
  else
  	invX = -1;
  if (spawnAreaY >= 0)
  	invY = 1;
  else
  	invY = -1;
  
  while(occupied){
  	cellX = tree[1].point.x/map.info.resolution + (rand() % spawnAreaX)*invX - spawnAreaX/4;
  	cellY = tree[1].point.x/map.info.resolution + (rand() % spawnAreaY)*invY - spawnAreaY/4;
  	printf("cellX:	%f\ncellY:	%f\n", cellX, cellY);
  	/*
  	if(map.data[cellX-floor(map.info.origin.position.x/map.info.resolution) + (cellY-1 - floor(map.info.origin.position.y/map.info.resolution))*map.info.width] < 5){
  		occupied = false;
  	}
  	*/
  	if(map.data[(int)((cellX-1-map.info.origin.position.x/map.info.resolution)*map.info.height) + (int)(cellY - map.info.origin.position.y/map.info.resolution)] < 5){
  		occupied = false;
  	}
  	
  }
  
  
  //convert pixels to coordinates
  /*
  newNode.point.x = cellX * map.info.resolution + map.info.origin.position.x;
  newNode.point.y = cellY * map.info.resolution + map.info.origin.position.y;
  */  
  
  newNode.point.x = cellX * map.info.resolution + tree[1].point.x;
  newNode.point.y = cellY * map.info.resolution + tree[1].point.y;
  
  printf("new Node #%i x=%f y=%f\n", nodeCounter, newNode.point.x, newNode.point.y);
}

int rrt::closestNode(){

  float closestDistance = 100000000.0f;
  float distanceTresh = 5;
  float distance;
  int closestNodeID = -1;
  

  for(int n=nodeCounter-1; n>=0; n--){
    distance = sqrt(pow(newNode.point.x-tree[n].point.x,2)+pow(newNode.point.y-tree[n].point.y,2));
    
 		if(distance < distanceTresh && tree[n].cost + distance < closestDistance && n!=0){
 		
    	if(collisionCheck(tree[n].point, newNode.point)==false){
      	closestDistance = tree[n].cost + distance;
     		closestNodeID = n;
     		printf("closestNodeID: %i\n",closestNodeID);
  			//TODO save n closest nodes in array for RRT* rewiring
  		}
    }    
    
  }
  
  //TODO for loop checking n closest nodes and calculate lowest cost connection. change closestNodeID to result
  
  if(closestNodeID > 0){
    tree[nodeCounter].point.x = newNode.point.x;
    tree[nodeCounter].point.y = newNode.point.y;
    tree[nodeCounter].parentID = closestNodeID;
    tree[nodeCounter].cost = tree[closestNodeID].cost + closestDistance;
    printf("node #%i cost=%f with node #%i\n", nodeCounter, tree[nodeCounter].cost, tree[nodeCounter].parentID);
    
    //last distance calculated is distance to goal. if close enough path is found
    if(distance < goalArea){
      pathFound = true;
      tree[0].parentID = nodeCounter;
      tree[0].cost = tree[nodeCounter].cost + distance;
      printf("path found. Distance is: %f\n", tree[0].cost);
    }
    return 0;
  }
  else{
  	printf("connection failed\n");	
  	return 1;
	}
}

bool rrt::collisionCheck(geometry_msgs::Point point1, geometry_msgs::Point point2){

	//TODO create array with linearly spaced points, with distance = size_robot. check for occupancy>constant in radius size_robot/2
	printf("checking for collision...	");
	float distance = sqrt(pow(point1.x-point2.x,2)+pow(point1.y-point2.y,2));
	int checkPoints = distance/robotSize;
	
	geometry_msgs::Point line[checkPoints+1];
	
	/*
	for(int n=0; n < checkPoints; n++){
		//add points along path into array in pixels
		line[n].x = (point1.x + n*(point1.x-point2.x)/checkPoints)*map.info.resolution; 
		line[n].y = (point1.y + n*(point1.y-point2.y)/checkPoints)*map.info.resolution;
	}
	
	//pull costmap -> already saved in map
	
	//convert costmap array into matrix	NVM calculate with mao directliy -> less operations
	int ko2arr; //saves 1D "koordinates" for 1D array
	
	for(int k=0; k<checkPoints; k++){
		ko2arr = (line[k].y-map.info.width) + (line[k].x);
		
		if(map.data[ko2arr] > 30){
			printf("collision detected\n");
			return true;
		}
		
		/*
		for(int j=0; j<searchRadius; j++){
		
			for(int i=0; i<searchRadius; i++){
				ko2arr = (line[k].y-robotSize*map.info.resolution/2.0f+j)*map.info.width;	//influence of y: which row
				ko2arr += (line[k].x-robotSize*map.info.resolution/2.0f+i); // influence of x: which column
				
				if(map.data[ko2arr]!=0){
					printf("collision detected\n");
					return true;
				}
			}
		}
		*don't forget to comment back when finished
		
	}
	*/
	printf("no collision\n");
	return false;
}

void rrt::createPath(){
  int nodesToGoal = 0;
  nodeCounter = 0;
  
  while(nodeCounter != 1){
    nodeCounter = tree[nodeCounter].parentID;
    nodesToGoal++;
  }
  
  std_msgs::Int16 msg;
  msg.data = nodesToGoal;
  printf("amount of nodes to goal is: %i\n", nodesToGoal);
  pubPathPointsNumber.publish(msg);
  
  nodeCounter = 0;

	visualization_msgs::Marker pathMarker;
	pathMarker.type = visualization_msgs::Marker::LINE_STRIP;
	pathMarker.header.frame_id = "map";
	pathMarker.ns = "path";
	pathMarker.id = 0;
	pathMarker.action = 0;
	
	pathMarker.color.r = 0.5f;
	pathMarker.color.g = 0.5f;
	pathMarker.color.a = 1.0f;
	pathMarker.scale.x = 0.4f;
	
	while(true){
		pathMarker.points.push_back(tree[nodeCounter].point);
		pathMarker.pose.position.z = 0.005f;
		if(nodeCounter==1)
			break;
		nodeCounter = tree[nodeCounter].parentID;
	}
	pubMarker.publish(pathMarker);
}

//Visualization

void rrt::marker(int markerNum){
	//Nodes
	visualization_msgs::Marker nodeMarker;
	nodeMarker.type = visualization_msgs::Marker::SPHERE;
	
	nodeMarker.header.frame_id = "map";
	nodeMarker.ns = "treepoint";
	nodeMarker.id = markerNum;
	
	nodeMarker.action = visualization_msgs::Marker::ADD;
	
	nodeMarker.color.g = 1.0f;
	if(markerNum==0){
		nodeMarker.color.r = 1.0f;
		nodeMarker.color.g = 0.0f;
	}
	if(markerNum==1){
		nodeMarker.color.b = 1.0f;
		nodeMarker.color.g = 0.0f;
	}
	nodeMarker.color.a = 1.0f;
	
	nodeMarker.scale.x = 0.4f;
	nodeMarker.scale.y = 0.4f;
	nodeMarker.scale.z = 0.1f;
	
	nodeMarker.pose.position = tree[markerNum].point;
	nodeMarker.pose.position.z = 0.01f;
	nodeMarker.pose.orientation.w = 1.0f;
	
	pubMarker.publish(nodeMarker);
	
	//Tree
	visualization_msgs::Marker treeMarker;
	treeMarker.type = visualization_msgs::Marker::LINE_LIST;
	nodeMarker.action = visualization_msgs::Marker::ADD;
	
	treeMarker.header.frame_id = "map";
	treeMarker.ns = "tree";
	treeMarker.id = markerNum;
	
	treeMarker.color.b = 1.0f;
	treeMarker.color.a = 1.0f;
	treeMarker.scale.x = 0.1f;
	treeMarker.pose.orientation.w = 1.0f;
	
	treeMarker.points.push_back(tree[markerNum].point);
	treeMarker.points.push_back(tree[tree[markerNum].parentID].point);
	
	pubMarker.publish(treeMarker);
	
}

void rrt::clearMarker(){

	visualization_msgs::Marker deleteAll;
	deleteAll.action = 3;
	pubMarker.publish(deleteAll);

}















