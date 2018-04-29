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
    void generateCmd();
    int closestNode();
    
    float goalArea;
    bool mapGoalExist;
    bool initialPoseExist;
    bool pathFound;
    
    nav_msgs::OccupancyGrid map;
    
    float distanceX;
  	float distanceY;
  	
  	void marker(int markerNum, int connection);
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
  marker(0,0);
}

void rrt::initialPoseCallback(const nav_msgs::Odometry& msg){
  if(initialPoseExist == false){
    tree[1].endPose = msg.pose.pose
    tree[1].parentID = -1;
    tree[1].cost = 0.0;
    
    initialPoseExist = true;
    printf("Initial Position is set to x=%f and y=%f\n", tree[1].point.x, tree[1].point.y);
  }
  marker(1,0);
}


void rrt::generateCmd(){
	
	//generate n command velocities which run for time t
	int n= 10;
	float angx;
	float angMax =  1;
	float speed = 1;
	
	geometry_msgs::Pose tempPose;
	geometry_msgs::Twist tempCmd;
	
	float closestDistance;
	float tempDistance;
	float cost;
	
	for(int counter=0; counter<n; counter++){
		angz = rand() % angMax;
	
	
	//integrate path to see determine path position after time t and distance travelled
		tempPose.position.x = tree[nodeCounter-1].endPose.position.x + speed*cos(tree[nodeCounter-1].pose.orientation.z);
		tempPose.position.y = tree[nodeCounter-1].endPose.position.y + speed*sin(tree[nodeCounter-1].pose.orientation.z);
	
	//choose path closest to goal
		tempDistance = sqrt(pow(tree[0].endPose.position.x - tempPose.position.x,2)+pow(tree[0].endPose.position.y - tempPose.position.y,2));
	
	//check if path end is actually closer than start
		if(tempDistance < closestDistance && tempDistance < tree[nodeCounter-1].distanceToGoal){
			closestDistance = tempDistance;
			tree[nodeCounter].cmd.linear.x = speed;
			tree[nodeCounter].cmd.angular.z = angz;	
			tree[nodeCounter].distanceToGoal = tempDistance;
			tree[nodeCounter].endPose = tempPose;
			
			//add visualization
		}
	}
}

bool rrt::collisionCheck(geometry_msgs::Point point1, geometry_msgs::Point point2){

}


//Visualization

void rrt::marker(int markerNum, int connection){
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
	
	if(true){
		visualization_msgs::Marker treeMarker;
		treeMarker.type = visualization_msgs::Marker::LINE_LIST;
		nodeMarker.action = visualization_msgs::Marker::ADD;
	
		treeMarker.header.frame_id = "map";
		treeMarker.ns = "tree";
		treeMarker.id = markerNum;
	
		treeMarker.color.b = 1.0f;
		treeMarker.color.a = 1.0f;
		
		if(connection == 1){
			treeMarker.color.b = 0.0f;
			treeMarker.color.r = 1.0f;
		}
	
		treeMarker.scale.x = 0.1f;
		treeMarker.pose.orientation.w = 1.0f;
	
		treeMarker.points.push_back(tree[markerNum].point);
		treeMarker.points.push_back(tree[tree[markerNum].parentID].point);
	
		pubMarker.publish(treeMarker);
	}
	
}

void rrt::clearMarker(){

	visualization_msgs::Marker deleteAll;
	deleteAll.action = 3;
	pubMarker.publish(deleteAll);

}















