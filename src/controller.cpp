#include <ros/ros.h>
#include "controller.h"
#include <tf/transform_datatypes.h>
#include "visualization_msgs/Marker.h"

void markerPoint(geometry_msgs::Pose, int);

int main(int argc, char **argv){

	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;
	controller robot_ctrl(nh);
	
	ros::Rate samplingRate(1/robot_ctrl.Ts);
	ros::Rate spinRate(60);
	
	float err, ctrl;
  geometry_msgs::Pose pathPose;

  while(ros::ok()){
    ros::spinOnce();
    
    if(robot_ctrl.cmdOk){
      robot_ctrl.pathStartSet(&pathPose);
      ros::Time start = ros::Time::now();
      
      while(robot_ctrl.cmdOk && ros::ok()){
        samplingRate.sleep();
        /*
        pathPose = robot_ctrl.pathPoseSet(pathPose);
        err = robot_ctrl.error(pathPose);
        ctrl = robot_ctrl.pid(err);
        printf("error: %f  controller output: %f\n", err, ctrl);
        robot_ctrl.cmdPub(ctrl);
        */
        
        ctrl = 0;
        pathPose = robot_ctrl.pathPoseSet(pathPose);
        ros::spinOnce();
        err = robot_ctrl.error(pathPose);
        robot_ctrl.pid(&robot_ctrl.ctrlDeviation);
        robot_ctrl.pid(&robot_ctrl.ctrlSpeed);
        robot_ctrl.pid(&robot_ctrl.ctrlAngle);
        //printf("error: %f  controller output: %f\n", err, ctrl);
        robot_ctrl.cmdPub(ctrl);
        
        
        if(ros::Time::now().toSec() - start.toSec() >= robot_ctrl.pubCounter + 1){ //TODO increase pubCounter when robot is at the end of first command
          robot_ctrl.pubCounter++;
          printf("%i\n", robot_ctrl.pubCounter);
        }
      }
    }
    //spinRate.sleep();
  }
}

//-----Callbacks------
//--------------------

void controller::cmdNumCallback(const std_msgs::Int16& msg){
  cmdNum = msg.data;
  printf("receiving %i commands...\n", cmdNum);
  pathCmds = new geometry_msgs::Twist[cmdNum];
  pathCmdsCpy = new geometry_msgs::Twist[cmdNum];
}

void controller::cmdCallback(const geometry_msgs::Twist& msg){
  if(cmdNum != 0){
    pathCmds[cmdCounter] = msg;
    printf("cmd #%i received\n", cmdCounter);
    cmdCounter++;
    
    if(cmdCounter == cmdNum){
     cmdCounter = 0;
     cmdOk = true;
     printf("path received. %i cmds in path\n", cmdNum);
    }
  }
}

void controller::cancelCallback(const std_msgs::Int16& msg){
	cmdCounter = 0;
	pubCounter = 0;
	cmdOk = false;
	cmdNum = 0;
	printf("path cancelled.\n");
}

void controller::odomCallback(const nav_msgs::Odometry& msg){
  currentOdom = msg;  
}

void controller::mapMetaCallback(const nav_msgs::MapMetaData& msg){
  mapMeta = msg;
}
//-----Functions------
//--------------------
void controller::pathStartSet(geometry_msgs::Pose* startPose){
  *startPose = currentOdom.pose.pose;
}

geometry_msgs::Pose controller::pathPoseSet(geometry_msgs::Pose startPose){
  //TODO: check multiple points and see which is the closest -> use that one to calculate d
  
  float w = pathCmds[pubCounter].angular.z;
  float v = pathCmds[pubCounter].linear.x;
  float alpha1 = tf::getYaw(startPose.orientation);
  
  float endX = startPose.position.x - v/w * sin(alpha1) + v/w * sin(alpha1 + w*Ts);
  float endY = startPose.position.y + v/w * cos(alpha1) - v/w * cos(alpha1 + w*Ts);
  float alpha2 = tf::getYaw(startPose.orientation) + w*Ts;
  
  geometry_msgs::Pose endPose;
  endPose.position.x = endX;
  endPose.position.y = endY;
  
  tf::Quaternion q_end = tf::createQuaternionFromRPY(0,0,alpha2);
  q_end.normalize();
	tf::quaternionTFToMsg(q_end, endPose.orientation);

  markerPoint(endPose, pubCounter);
	
  return endPose;
}
/*
float controller::error(geometry_msgs::Pose pathPose){
  ros::spinOnce();
  float dx = pathPose.position.x - currentOdom.pose.pose.position.x;
  float dy = pathPose.position.y - currentOdom.pose.pose.position.y;
  float d = sqrt(dx*dx+dy*dy);
  float alpha = atan2(dy,dx) - tf::getYaw(currentOdom.pose.pose.orientation);
  //int sign = (alpha > 0) - (alpha < 0);
  float error = d * sin(alpha)/10;
  
  error = Ts*(tf::getYaw(pathPose.orientation) - tf::getYaw(currentOdom.pose.pose.orientation)); //Only compares angles
  return error;
}*/

float controller::error(geometry_msgs::Pose pathPose){ //error for angle, deviation and speed
  float x, xr, y ,yr, theta, thetar;
  
  xr = pathPose.position.x;
  yr = pathPose.position.y;
  thetar = tf::getYaw(pathPose.orientation);
  
  x = currentOdom.pose.pose.position.x;
  y = currentOdom.pose.pose.position.y;
  theta = tf::getYaw(currentOdom.pose.pose.orientation);
  
  float e1, e2, e3;
  
  e1 = -cos(theta) * (x-xr) - sin(theta) * (y-yr);
  e2 = sin(theta) * (x-xr) - cos(theta) * (y-yr);
  e3 = -(theta-thetar);
  
  float ur1, ur2, ue1, ue2, k1, k2, k3, xi, g, sign_ur1;
  
  xi = 0.6;
  g = 2;
  ur1 = pathCmds[pubCounter].linear.x;
  ur2 = pathCmds[pubCounter].angular.z;
  
  k1 = 2 * xi * sqrt(ur2*ur2 + g*ur1*g*ur1);
  k2 = g * abs(ur1);
  k3 = k1;
  
  sign_ur1 = (ur1 > 0) - (ur1 < 0);
  ue1 = -k1 * e1;
  ue2 = -sign_ur1 * k2 * e2 - k3 * e3;
  
  float v, w;
  
  v = cos(e3) * ur1 - ue1;
  w = ur2 - ue2;
  
  pathCmdsCpy[pubCounter].linear.x = v;
  pathCmdsCpy[pubCounter].angular.z = w;
  
  return 0;
}
/*
float controller::pid(float error){
	I = I + Ts*error;
	D = (error-oldError)/Ts;
	P = error;
	oldError = error;
	float ctrl = Kp*P + Ki*I + Kd*D;
	return ctrl;
}*/

float controller::pid(controlValue* control){
  control->P = control->error;
	control->I = control->I + Ts * control->error;
	control->D = (control->error - control->oldError) / Ts;

	control->ctrl = control->Kp * control->P + control->Ki * control->I + control->Kd * control->D;
	return 0.0f;
}

//-----Publishers-----
//--------------------
/*
void controller::cmdPub(float ctrl){
  pathCmds[pubCounter].angular.z += ctrl;
  pubCmdVel.publish(pathCmds[pubCounter]);
  
  if(pubCounter == cmdNum){
    pubCounter = 0;
    cmdOk = false;
    printf("publishing finished.\n");
  }
}*/

void controller::cmdPub(float ctrl){
  pathCmdsCpy[pubCounter].angular.z;
  pathCmdsCpy[pubCounter].linear.x;
  pubCmdVel.publish(pathCmds[pubCounter]);
  //printf("ctrlDev: %f   ctrlSpeed: %f   ctrlAngle: %f\n", ctrlDeviation.ctrl, ctrlSpeed.ctrl, ctrlAngle.ctrl);
  printf("linear x: %f   angular z: %f\n", pathCmds[pubCounter].linear.x, pathCmds[pubCounter].angular.z);
    
  if(pubCounter == cmdNum){
    pubCounter = 0;
    cmdOk = false;
    printf("publishing finished.\n");
  }
}


void markerPoint(geometry_msgs::Pose pose, int id){
  //type: 0-goal, 1-start, 2-generic, 3-mini
  ros::NodeHandle nh;
  ros::Publisher pubMarker;
  pubMarker = nh.advertise<visualization_msgs::Marker>("treepoints",50,true);
  
	visualization_msgs::Marker nodeMarker;
	nodeMarker.type = visualization_msgs::Marker::SPHERE;
	
	nodeMarker.header.frame_id = "map";
	nodeMarker.ns = "treepoint";
	nodeMarker.id = id;
	
	nodeMarker.action = visualization_msgs::Marker::ADD;
	
	nodeMarker.color.b = 1.0f;
	nodeMarker.color.a = 1.0f;
	
	nodeMarker.scale.x = 0.2f;
	nodeMarker.scale.y = 0.2f;
	nodeMarker.scale.z = 0.01f;
	
	nodeMarker.pose = pose;
	nodeMarker.pose.position.z = 0.01f;
	nodeMarker.pose.orientation.w = 1.0f;
	
	if(id==0){
		nodeMarker.color.r = 1.0f;
		nodeMarker.color.b = 0.0f;
	}
	if(id==1){
		nodeMarker.color.g = 1.0f;
		nodeMarker.color.b = 0.0f;
	}

	pubMarker.publish(nodeMarker);	
}







