#include <ros/ros.h>

//msgs
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

class tree{
	public:
    tree(ros::NodeHandle);
    ~tree(){}
    
    int pointsInTree;
    bool mapFound;
		bool initialPoseFound;
		bool goalFound;
		bool pathFound;
		
		geometry_msgs::Point goal;
		
		void generatePoint();
		int generateCommand(geometry_msgs::Point goal);
		int cmdIntegration(float speed, geometry_msgs::Pose start, geometry_msgs::Twist cmd, geometry_msgs::Pose* end, float* cost);
		bool rrt::collisionCheck(geometry_msgs::Point point);
    	
	private:
	   struct treePose{
    	int id;
    	geometry_msgs::Pose pose;
    	geometry_msgs::Twist cmd;
    	float cost;
    	
    	tree* parent;
    	treePose();
    };
		
		nav_msgs::OccupancGrid map;
		int maxPoints;
		treePose* treePoints;
		float maxPointDistance = 1.0f;
		int interpolationSteps = 10;
		
		ros::Subscriber subMap;
		ros::Subscriber subPose;
		ros::Subscriber subGoal;
		
		void mapCallback(const nav_msgs::OccupanceGrid& map_msg);
		void currentPoseCallback(const geometry_msgs::Pose& pose_msg);
		void goalCallback(const geometry_msgs::Point& goal_msg);
};

//---Callback functions---
//------------------------

void tree::mapCallback(const nav_msgs::OccupanceGrid& msg);
/*
save costmap to map in tree class
*/

void tree::currentPoseCallback(const geometry_msgs::Pose& msg);
/*
save current position into treePoints with id=1, parent=NULL
*/

void tree::goalCallback(const geometry_msgs::Point& msg);
/*
save current position into treePoints with id=0, parent=NULL
*/


//-------Functions--------
//------------------------
float distance(geometry_msgs::Point point1, geometry_msgs::Point point2);
/*
returns distance between two points
*/

void generatePoint();
/* 
generates and returns random Point. sampling area is determined by the points in tree.
*/

int generateCommand(geometry_msgs::Pose* origin, geometry_msgs::Point* goal, geometry_msgs::Twist* cmd);
/*
generates commands, saves in cmd. only short distances-> generates only one command for set amount of time
returns 0 when collisionfree command found.

Args:
	-origin: current pose/pose in tree of robot
	-point it needs to reach
*/

bool collisionCheck(geomemtry_msgs::Point point);
/*
checks occupancy Grid at point. returns true if point is blocked

Args:
	point: point to check
*/


//-----Visualization------
//------------------------
void markerPoint(geometry_msgs::Pose pose, int type){
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
	
	nodeMarker.scale.x = 0.2f;
	nodeMarker.scale.y = 0.2f;
	nodeMarker.scale.z = 0.01f;
	
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

