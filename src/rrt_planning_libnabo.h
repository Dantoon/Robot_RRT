#include <ros/ros.h>
#include "nabo/nabo.h"

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
		
		void generatePoint();
		int generateCommand(Eigen::VectorXf, int startId);
		int cmdIntegration(geometry_msgs::Twist, geometry_msgs::Quaternion*, Eigen::Vector2f*, float*);
		bool collisionCheck(Eigen::Vector2f);
		float vectorDistance(Eigen::Vector2f, Eigen::Vector2f);
		Eigen::Vector2f cellToCoord(int cell);
		int coordToCell(Eigen::Vector2f);
		void pathCmd();
    
    //replanning
    geometry_msgs::Point closestPoint1;
    geometry_msgs::Point closestPoint2;
    float devTolerance;
    void devCheck(nav_msgs::Odometry);
    //void dynCollisionCheck();
    
    //visualization
    void markerPoint(Eigen::Vector2f, int type);
    void clearMarker();
    void createPath();
    void markerStrip(Eigen::Vector2f, int type);
    void markerList(int markerNum);
    visualization_msgs::Marker treeMarker;
    
    //libnabo stuff
    Eigen::MatrixXf treePositions;
    Eigen::Vector2f goal;
    	
	private:
	  struct treeCmd{
    	int id;
    	int parentId;
    	geometry_msgs::Twist cmd;
    	geometry_msgs::Quaternion orientation;
    	float cost;
    	
    	treeCmd();
    };
		
		nav_msgs::OccupancyGrid map;
		treeCmd* treeCmds;
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
		void initialPoseCallback(const nav_msgs::Odometry&);
		void goalCallback(const geometry_msgs::Point&);
		void replanCallback(const std_msgs::Bool&);
};

//------Constructors------
//------------------------

tree::tree(ros::NodeHandle nh){
	pointsInTree = 1;
	mapFound = false;
	initialPoseFound = false;
	goalFound = false;
	pathFound = false;
	
	maxPoints = 10000;
	treeCmds = new treeCmd[maxPoints];
	//temporary. testing what happens when no limit on sampling distance TODO
	maxPointDistance = 1.0f;
	interpolationSteps = 10;
	
	//pubCmd = nh.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel",50,false);	
	pubCmd = nh.advertise<geometry_msgs::Twist>("/rrt_cmd",50,false);	
	pubProjection = nh.advertise<geometry_msgs::Twist>("/cmds",500,false);	
  pubMarker = nh.advertise<visualization_msgs::Marker>("treepoints",50,false);
  pubCmdNum = nh.advertise<std_msgs::Int16>("/cmdNum",50,false);
  
  subMap = nh.subscribe("robot_0/robot_map/robot_map/costmap", 10, &tree::mapCallback, this);
  subPose = nh.subscribe("robot_0/odom", 10, &tree::initialPoseCallback, this);
  subGoal = nh.subscribe("map_goal", 10, &tree::goalCallback, this);
  subReplan = nh.subscribe("/rrt_replan", 10, &tree::replanCallback, this);
  
  //libnabo stuff
  treePositions.resize(2, maxPoints);
}

tree::treeCmd::treeCmd(){
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
void tree::markerPoint(Eigen::Vector2f position, int type){
  //type: 0-goal, 1-start, 2-generic, 3-mini
  
	visualization_msgs::Marker nodeMarker;
	nodeMarker.type = visualization_msgs::Marker::SPHERE;
	
	nodeMarker.header.frame_id = "map";
	nodeMarker.ns = "treePoint";
	nodeMarker.id = type;
	
	nodeMarker.action = visualization_msgs::Marker::ADD;
	
	nodeMarker.color.b = 1.0f;
	nodeMarker.color.a = 1.0f;
	
	nodeMarker.scale.x = 0.2f;
	nodeMarker.scale.y = 0.2f;
	nodeMarker.scale.z = 0.01f;
	
	nodeMarker.pose.position.x = position(0);
	nodeMarker.pose.position.y = position(1);
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

void tree::createPath()
{
  int tempId = 0;
  treeCmd tempCmd = treeCmds[pointsInTree];
  markerStrip(treePositions.col(0), 1);
  while(tempId != pointsInTree)
  {
    while(tempCmd.parentId != tempId)
      tempCmd = treeCmds[tempCmd.parentId];
    tempId = tempCmd.id;
    markerStrip(treePositions.col(tempId), 1);
  }
}

void tree::markerStrip(Eigen::Vector2f position, int type){
  //type: 0-final path, 1-generic 
	treeMarker.type = visualization_msgs::Marker::LINE_STRIP;
	treeMarker.action = visualization_msgs::Marker::ADD;

	treeMarker.header.frame_id = "map";
	treeMarker.ns = "treeStrip";
	treeMarker.id = 1;

	treeMarker.color.r = 1.0f;
	treeMarker.color.a = 1.0f;

	treeMarker.scale.x = 0.05f;
	treeMarker.pose.orientation.w = 1.0f;
  
  geometry_msgs::Point p;
  p.x = position(0);
  p.y = position(1);
  p.z = 0.1f;
	treeMarker.points.push_back(p);

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
	
	geometry_msgs::Point p1, p2;
	p1.x = treePositions(0, markerNum);
	p1.y = treePositions(1, markerNum);
	p1.z = 0;
	p2.x = treePositions(0, treeCmds[markerNum].parentId);
	p2.y = treePositions(1, treeCmds[markerNum].parentId);
	p2.z = 0;

	listMarker.points.push_back(p1);
	listMarker.points.push_back(p2);

	pubMarker.publish(listMarker);
}
