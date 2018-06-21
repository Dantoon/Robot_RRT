#include <ros/ros.h>

//msgs
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"

class tree{

	public:
    tree(ros::NodeHandle);
    ~tree(){}
    
    int pointsInTree;
		int maxPoints;
    bool mapFound;
		bool initialPoseFound;
		bool goalFound;
		bool pathFound;
		
		geometry_msgs::Point goal;
		
		void generatePoint();
		int generateCommand(geometry_msgs::Point goal, int startId);
		int cmdIntegration(float speed, geometry_msgs::Pose start, geometry_msgs::Twist cmd, geometry_msgs::Pose* end, float* cost);
		bool collisionCheck(geometry_msgs::Point point);
		float distance(geometry_msgs::Point point1, geometry_msgs::Point point2);
		geometry_msgs::Point cellToCoord(int cell);
		int coordToCell(geometry_msgs::Point coord);
		void pathCmd();
    
    //replanning
    geometry_msgs::Point closestPoint1;
    geometry_msgs::Point closestPoint2;
    float devTolerance;
    void devCheck(nav_msgs::Odometry);
    //void dynCollisionCheck();
    
    //visualization
    void markerPoint(geometry_msgs::Pose pose, int type);
    void clearMarker();
    void createPath();
    void markerLine(geometry_msgs::Pose pose, int type);
    void markerList(int markerNum);
    visualization_msgs::Marker treeMarker;
    	
	private:
	  struct treePose{
    	int id;
    	geometry_msgs::Pose pose;
    	geometry_msgs::Twist cmd;
    	float cost;
    	
    	int parentId;
    	treePose();
    };
		
		nav_msgs::OccupancyGrid map;
		treePose* treePoints;
		float maxPointDistance;
		int interpolationSteps;
		
		ros::Subscriber subMap;
		ros::Subscriber subPose;
		ros::Subscriber subGoal;
		ros::Subscriber subReplan;
		
		ros::Publisher pubMarker;
		ros::Publisher pubCmd;
		ros::Publisher pubProjection;
		ros::Publisher pubCmdNum;
		
		void mapCallback(const nav_msgs::OccupancyGrid&);
		void currentPoseCallback(const nav_msgs::Odometry&);
		void goalCallback(const geometry_msgs::Point&);
		void replanCallback(const std_msgs::Bool&);
};

//------Constructors------
//------------------------

tree::tree(ros::NodeHandle nh){
	pointsInTree = 2;
	mapFound = false;
	initialPoseFound = false;
	goalFound = false;
	pathFound = false;
	
	maxPoints = 10000;
	treePoints = new treePose[maxPoints];
	maxPointDistance = 1.0f;
	interpolationSteps = 10;
	
	//pubCmd = nh.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel",50,false);	
	pubCmd = nh.advertise<geometry_msgs::Twist>("/rrt_cmd",50,false);	
	pubProjection = nh.advertise<geometry_msgs::Twist>("/cmds",500,false);	
  pubMarker = nh.advertise<visualization_msgs::Marker>("treepoints",50,false);
  pubCmdNum = nh.advertise<std_msgs::Int16>("/cmdNum",50,false);
  
  subMap = nh.subscribe("robot_0/robot_map/robot_map/costmap", 10, &tree::mapCallback, this);
  subPose = nh.subscribe("robot_0/odom", 10, &tree::currentPoseCallback, this);
  subGoal = nh.subscribe("map_goal", 10, &tree::goalCallback, this);
  subReplan = nh.subscribe("/rrt_replan", 10, &tree::replanCallback, this);
}

tree::treePose::treePose(){
	cmd.linear.x = 0.0f;
	cmd.linear.y = 0.0f;
	cmd.linear.z = 0.0f;
	cmd.angular.x = 0.0f;
	cmd.angular.y = 0.0f;
	cmd.angular.z = 0.0f;

	parentId = 0;
}

//-------Functions--------
//------------------------
//float distance(geometry_msgs::Point point1, geometry_msgs::Point point2);
/*
returns distance of two points in xy-plane
*/

//-----Visualization------
//------------------------
void tree::markerPoint(geometry_msgs::Pose pose, int type){
  //type: 0-goal, 1-start, 2-generic, 3-mini
  
	visualization_msgs::Marker nodeMarker;
	nodeMarker.type = visualization_msgs::Marker::SPHERE;
	
	nodeMarker.header.frame_id = "map";
	nodeMarker.ns = "treepoint";
	nodeMarker.id = type;
	
	nodeMarker.action = visualization_msgs::Marker::ADD;
	
	nodeMarker.color.b = 1.0f;
	nodeMarker.color.a = 1.0f;
	
	nodeMarker.scale.x = 0.2f;
	nodeMarker.scale.y = 0.2f;
	nodeMarker.scale.z = 0.01f;
	
	nodeMarker.pose = pose;
	nodeMarker.pose.position.z = 0.01f;
	nodeMarker.pose.orientation.w = 1.0f;
	
	if(type==0){
		nodeMarker.color.r = 1.0f;
		nodeMarker.color.b = 0.0f;
	}
	if(type==1){
		nodeMarker.color.g = 1.0f;
		nodeMarker.color.b = 0.0f;
	}

	pubMarker.publish(nodeMarker);	
}

void tree::clearMarker(){

	visualization_msgs::Marker deleteAll;
	deleteAll.action = 3;
	pubMarker.publish(deleteAll);

}

void tree::createPath(){
  printf("creating Path...\n");
  treePose marker = treePoints[0];
  ros::Rate markerRate(100);
  
  while(true){
    printf("connecting point #%i to #%i\n", marker.id, marker.parentId);
    markerLine(marker.pose, 1);
    
    if(marker.id == 1){
      printf("test1\n");
      break;
    }
    printf("test2\n");      
    marker = treePoints[marker.parentId];
    markerRate.sleep();
	}
}

void tree::markerLine(geometry_msgs::Pose pose, int type){
  //type: 0-final path, 1-generic 
	treeMarker.type = visualization_msgs::Marker::LINE_STRIP;
	treeMarker.action = visualization_msgs::Marker::ADD;

	treeMarker.header.frame_id = "map";
	treeMarker.ns = "treeLine";
	treeMarker.id = 1;

	treeMarker.color.r = 1.0f;
	treeMarker.color.a = 1.0f;

	treeMarker.scale.x = 0.05f;
	treeMarker.pose.orientation.w = 1.0f;
  pose.position.z = 0.1f;

	treeMarker.points.push_back(pose.position);

	pubMarker.publish(treeMarker);
}

void tree::markerList(int markerNum){

	visualization_msgs::Marker listMarker;
	listMarker.type = visualization_msgs::Marker::LINE_LIST;
	listMarker.action = visualization_msgs::Marker::ADD;

	listMarker.header.frame_id = "map";
	listMarker.ns = "treeList";
	listMarker.id = markerNum;

	listMarker.color.b = 1.0f;
	listMarker.color.a = 1.0f;

	listMarker.scale.x = 0.05f;
	listMarker.pose.orientation.w = 1.0f;

	listMarker.points.push_back(treePoints[markerNum].pose.position);
	listMarker.points.push_back(treePoints[treePoints[markerNum].parentId].pose.position);

	pubMarker.publish(listMarker);
}
