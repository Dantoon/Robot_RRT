#include "tracker.h"
#include "tf/transform_datatypes.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "tracker");
  ros::NodeHandle nh;
  geometry_msgs::Point tempPoint;
  int tempID;
  int samplingRate = 2;
  ros::Rate spinRate(samplingRate);
  ros::Publisher pubObstaclePath;
  pubObstaclePath = nh.advertise<nav_msgs::Odometry>("/tracking", 50, false);
  
  callback callback(nh);
  while(!callback.mapOk)
    ros::spinOnce();
  
  while(ros::ok()){
    ros::spinOnce();
    //while spinning, checks objects for tracking and adds their position to structs if necessary
    
    trackedObject(callback.head, &pubObstaclePath, samplingRate);
    //calculates and publishes all positions and headings as odometry.
    if(callback.head->next->id == -1)
      printf("no objects\n");
    else{
      int counter = 0;
      callback.tempHead = callback.head->next;
      while(true){
        if(callback.tempHead->id == -1)
          break;
        counter++;
        callback.tempHead = callback.tempHead->next;
      }
      printf("%i object(s) tracked\n", counter);
    }
    //TODO finish -lineCollisionCheck-, odomcalc, subscriber of odoms (in forward projection?)
    
    
    spinRate.sleep();
  }
  
  return 0;
}

//----Constructors----
//--------------------
callback::callback(ros::NodeHandle nh){
  robotViewAngle = 2.35837626457;
  robotMinRange = 0.0;
  robotMaxRange = 20.0;
  
  startObstacle.next = &endObstacle;
  startObstacle.id = 0;
  endObstacle.next = NULL;
  endObstacle.id = -1;
  head = &startObstacle;
  tempHead = head;
  
  subOdom = nh.subscribe("/robot_0/odom", 1, &callback::odomCallback,this);
  subObstacle1_Odom = nh.subscribe("/robot_1/odom", 1, &callback::obstacle1_OdomCallback, this);
  subMap = nh.subscribe("/robot_0/robot_map/robot_map/costmap", 10, &callback::mapCallback, this);
}

obstacle::obstacle(){
  id = 0;
  pointNum = 0;
  next = NULL;
}

//-----Callback-------
//--------------------
void callback::odomCallback(const nav_msgs::Odometry& msg){
  robotPose = msg.pose.pose;
}

void callback::obstacle1_OdomCallback(const nav_msgs::Odometry& msg){
	if(mapOk){
  	geometry_msgs::Point position = msg.pose.pose.position;
  	bool inView = false;

  	float range = pointDistance(robotPose.position, position);
  	float angle = atan2(position.y-robotPose.position.y, position.x-robotPose.position.x);
  	float relativeAngle = angle - tf::getYaw(robotPose.orientation);
  	//printf("relative angle:\t\t%f\nrobot view angle:\t%f\n", fabs(relativeAngle), robotViewAngle);
  	bool collision = lineCollisionCheck(position, robotPose.position);

  	if(range < robotMaxRange && range > robotMinRange && fabs(relativeAngle) < robotViewAngle && !collision){
  	  inView = true;
  	}
	
  	trackingUpdate(1, inView, position, &head, &tempHead);
	}
}

void callback::mapCallback(const nav_msgs::OccupancyGrid& msg){
  map = msg;
  mapOk = true;
  printf("Map received.\n");
}

//-----Obstacle-------
//--------------------
void obstacle::addNewPoint(geometry_msgs::Point p){
  if(pointNum == 0){
    lastPoints[0] = p;
    pointNum = 1;
  }
  else if(pointNum == 1){
    lastPoints[1] = p;
    pointNum = 2;
  }
  else if(pointNum == 2){
    lastPoints[2] = p;
    pointNum = 3;
  }
  else if(pointNum == 3){
    lastPoints[0] = lastPoints[1];
    lastPoints[1] = lastPoints[2];
    lastPoints[2] = p;
  }
  printf("pointNum = %i\n", pointNum);
}

//-----Functions------
//--------------------

bool trackedObject(obstacle* ptr, ros::Publisher* pubPtr, int samplingRate){
  while(ptr->id != -1){
    printf("id: %i\n", ptr->id);
    if(ptr->pointNum == 3){
      printf("Object #%i full. publishing odom...\n", ptr->id);
      ptr->calculatedOdom = odomCalc(ptr->lastPoints[0], ptr->lastPoints[1], ptr->lastPoints[2], samplingRate);
      pubPtr->publish(ptr->calculatedOdom);
    }
    ptr = ptr->next;
  }
  
  return false;
}

void trackingUpdate(int id, bool inView, geometry_msgs::Point p, obstacle** head, obstacle** tempHead){
  obstacle* ptr = *head;
  bool objExist = false;
  //check if object with id exists in linked list. 
  //if in view & exist -> add point to obj
  //if in view & !exist-> create new object at start of linked list
  //if !in view & exist -> delete obj
  //if !in view % !exist-> nothing
  if(inView){
    printf("obstacle in view\n");
    while(true){
      printf("searching id #%i...\n", ptr->id);
      if(ptr->id == id){
        printf("adding to new object #%i...\n", ptr->id);
        ptr->addNewPoint(p);
        objExist = true;
        break;
      }
      ptr = ptr->next;
      if(ptr->id == -1)
        break;
    }
    
    if(!objExist){
      printf("creating new object...\n");
      *tempHead = new obstacle;
      (*tempHead)->next = (*head)->next;
      (*head)->next = *tempHead;
      (*tempHead)->id = id;
      (*tempHead)->addNewPoint(p);
    }
  }
  
  else{
    obstacle* oldPtr = ptr; 
    while(true){
      if(ptr->id == id){
        /*if(*head == ptr){
          //If to be deleted object is first in list, head needs to be changed.
          *head = ptr->next;
          printf("deleting object only\n");
          delete ptr;
        }*/ //head always points towards start obstacle object
        //else{
        obstacle* delObj = oldPtr->next;
        oldPtr->next = delObj->next;
        printf("deleting object id=%i\n", delObj->id);
        free(delObj);
          //better solution for later may be extrapolating position in this sampling step, until object comes back into view
       // }
        break;
      }
      oldPtr = ptr; 
      ptr = ptr->next;
      if(ptr->id == -1)
        break;
    }   
  }
}

nav_msgs::Odometry odomCalc(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point p3, int samplingRate){
  nav_msgs::Odometry odom;
  float alpha1, alpha2;
  float dx1, dx2, dy1, dy2, d1, d2;
  float pi = 3.14159265359;
  bool colinear = false; //set false if you want to take rotatin into account as well and it has already been implemented
  
  odom.twist.twist.linear.x = 0;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.linear.z = 0;
  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = 0;
  odom.pose.pose.position = p3;
  
  dx1 = p2.x-p1.x;
  dx2 = p3.x-p2.x;
  dy1 = p2.y-p1.y;
  dy2 = p3.y-p2.y;
  d1 = sqrt(dx1*dx1 + dy1*dy1);
  d2 = sqrt(dx2*dx2 + dy2*dy2);
  
  alpha1 = atan2(dy1, dx1);
  alpha2 = atan2(dy2, dx2);
  if(abs(alpha1-alpha2) < 0.05)
    colinear = true;
  
  if(colinear){
    odom.twist.twist.linear.x = pointDistance(p3,p1)*(double)(samplingRate)/2;
    odom.twist.twist.angular.z = 0;
    tf::Quaternion q = tf::createQuaternionFromRPY(0,0,alpha2);
    q.normalize();
    tf::quaternionTFToMsg(q, odom.pose.pose.orientation);
  }
  
  else{
    /*
    1. calc midpoint of circle
    2. calc angular velocity of robot
    3, calc speed of robot
    */
    float beta = 0.5*(pi+alpha1-alpha2);
    float d_avg = 0.5*(d1+d2);
    float r = d_avg/(2*cos(beta));
    float theta = 2*(pi-2*beta);
    float alphaEnd = alpha2-(pi/2.0-beta)+theta/2.0;
    
    odom.twist.twist.angular.z = theta*samplingRate;
    odom.twist.twist.linear.x = r*odom.twist.twist.angular.z;
    tf::Quaternion q = tf::createQuaternionFromRPY(0,0,alphaEnd);
    q.normalize();
    tf::quaternionTFToMsg(q, odom.pose.pose.orientation);
  }
  
  return odom;
}

bool callback::lineCollisionCheck(geometry_msgs::Point p1, geometry_msgs::Point p2){
  float distance = pointDistance(p1, p2);
  int resolutionFactor = 2;
  int checkPoints = (int)(distance)*resolutionFactor;
  float angle = atan2(p2.y-p1.y, p2.x-p1.x);
  geometry_msgs::Point tempPoint;
  
  for(int n = 0; n<=checkPoints; n++){
    tempPoint.x = p1.x + cos(angle)*n*distance/checkPoints;
    tempPoint.y = p1.y + sin(angle)*n*distance/checkPoints;
    if(collisionCheck(tempPoint))
      return true;
  }
  
  return false;
}

bool callback::collisionCheck(geometry_msgs::Point point){
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
  //printf("cellx = %i    celly = %i\n", cellx, celly);
  float value = map.data[cellx + celly*map.info.width];
  
  //printf("value: %f\n", value);
  if(value > 20.0){
    return true;
    //printf("collision detected\n");
  }
  return collision;
}

float pointDistance(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return sqrt( pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

