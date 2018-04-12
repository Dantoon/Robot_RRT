#include <ros/ros.h>
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"

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
    void closestNode();
    void createPath();
    void marker();
    
    float goalArea;
    bool mapGoalExist;
    bool initialPoseExist;
    bool pathFound;
    
    nav_msgs::OccupancyGrid map;
    
    /*
    int mapWidth;
    int mapHeight;
    float mapResolution;
    float mapPoseX;
    float mapPoseY;
    */
    
  private:
    ros::Subscriber subGoal;
    ros::Subscriber subMap;
    ros::Subscriber subMapMeta;
    ros::Subscriber subPose;
    ros::Publisher pubPath;
    ros::Publisher pubMarker;
    
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
  
  
  subGoal = nh.subscribe("map_goal", 10, &rrt::goalCallback, this);
  subPose = nh.subscribe("robot_0/odom", 10, &rrt::initialPoseCallback, this);
  subMap = nh.subscribe("map", 10, &rrt::mapCallback, this);
  pubPath = nh.advertise<nav_msgs::Path>("path", 50, true);
  pubMarker = nh.advertise<visualization_msgs::Marker>("treepoints",50,true);
}

void rrt::mapCallback(const nav_msgs::OccupancyGrid& msg){

  map = msg;
  printf("map detected\n");
  //TODO: rrt algorithm using map from /map
  
  
  //after succesful rrt, path gets published to /path
  
  nav_msgs::Path final_path;
  
  //pubPath.publish(final_path);

}

/*
void rrt::mapMetaCallback(const nav_msgs::MapMetaData& msg){

  mapWidth = msg.width;
  mapHeight = msg.height;
  mapResolution = msg.resolution;
  mapPoseX = msg.origin.position.x;
  mapPoseY = msg.origin.position.y;
  
  printf("width=%i height=%i res=%f poseX=%f poseY=%f\n", mapWidth, mapHeight, mapResolution, mapPoseX, mapPoseY);

}
*/

void rrt::goalCallback(const geometry_msgs::Twist& msg){
  tree[0].point.x = msg.linear.x;
  tree[0].point.y = msg.linear.y;
  
  printf("Map Goal is set to x=%f and y=%f\n", tree[0].point.x, tree[0].point.y);
  mapGoalExist = true;
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
}


void rrt::generateNode(){

  float x;
  float y;
  bool occupied = true;
  
  while(occupied){
    x = rand() % map.info.width;
    y = rand() % map.info.height;
    if(map.data[x*map.info.width + y] == 0){
      occupied = false;
    }
  }
  
  
  //convert pixels to coordinates
  x = x * map.info.resolution + map.info.origin.position.x;
  y = y * map.info.resolution + map.info.origin.position.y;
  
  newNode.point.x = x;
  newNode.point.y = y;
  //TODO: Check if new node is valid(free area? collision with edge?), connect it and increment counter. diff function?
  //also convert from pixels into koord.
  
  printf("new Node #%i x=%f y=%f\n", nodeCounter, newNode.point.x, newNode.point.y);  
}

void rrt::closestNode(){

  float closestDistance = 10000.0;
  float distance;
  int closestNodeID = -1;

  for(int n=nodeCounter-1; n>=0; n--){
    distance = sqrt(pow(newNode.point.x-tree[n].point.x,2)+(newNode.point.y-tree[n].point.y,2));
    if(distance < closestDistance && n!=0){
      closestDistance = distance;
      closestNodeID = n;
    }
  }
  
  if(closestNodeID != -1){
    tree[nodeCounter].point.x = newNode.point.x;
    tree[nodeCounter].point.y = newNode.point.y;
    tree[nodeCounter].parentID = closestNodeID;
    tree[nodeCounter].cost = tree[closestNodeID].cost + closestDistance;
    printf("node #%i cost=%f\n", nodeCounter, tree[nodeCounter].cost);
    
    //last distance calculated is distance to node. if close enough path is found
    if(distance < goalArea){
      pathFound = true;
      tree[0].parentID = closestNodeID;
      tree[0].cost = tree[nodeCounter].cost + distance;
      printf("path found. Distance is: %f\n", tree[0].cost);
    }
    
    nodeCounter++;
  }
}

void rrt::createPath(){
  int nodesToGoal = 0;
  nodeCounter = 0;
  
  while(nodeCounter != 1){
    nodeCounter = tree[nodeCounter].parentID;
    nodesToGoal++;
  }
  
  printf("amount of nodes to goal is: %i\n", nodesToGoal);
  
  //segmentation faul core dumped. Due to poses[] not having enough places in array to save values to? how to initializ size?

  //geometry_msgs::PoseStamped poses[nodesToGoal];
  nodeCounter = 0;
  
  nav_msgs::Path myPath;
  //geometry_msgs::PoseStamped myPath.poses[nodesToGoal];
  
  for(int n = nodesToGoal-1; n>=0; n-- ){
    myPath.poses[n].pose.position = tree[nodeCounter].point;
    //poses[n].pose.position = tree[nodeCounter].point;
    nodeCounter = tree[nodeCounter].parentID;
  }
	
  //myPath.poses = poses;
  pubPath.publish(myPath);
  
}

//check for obstacle collision with edges?
      //check for collision using linspace like function?
      //-> create vector and check if any of the points are on occupied area
      
//Visualization

void rrt::marker(){
	visualization_msgs::Marker marker;
	marker.type = visualization_msgs::Marker::SPHERE;
	
	marker.header.frame_id = "treepoints";
	marker.ns = "treepoints";
	
	marker.color.g = 1.0;
	marker.color.a = 1.0;
	
	marker.scale.x = 5;
	marker.scale.y = 5;
	marker.scale.z = 1;
	
	marker.pose.position = newNode.point;
	pubMarker.publish(marker);
	
}

















