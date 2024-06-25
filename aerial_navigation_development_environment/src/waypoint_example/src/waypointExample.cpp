#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

string waypoint_file_dir;
double waypointXYRadius = 2.0;
double waypointZBound = 5.0;
double waitTime = 0;
double waitTimeStart = 0;
bool isWaiting = false;
double frameRate = 5.0;
double speed = 2.0;
bool sendSpeed = true;

pcl::PointCloud<pcl::PointXYZ>::Ptr waypoints(new pcl::PointCloud<pcl::PointXYZ>());

float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
double curTime = 0, waypointTime = 0;

// reading waypoints from file function
void readWaypointFile()
{
  FILE* waypoint_file = fopen(waypoint_file_dir.c_str(), "r");
  if (waypoint_file == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(waypoint_file, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(waypoint_file, "%d", &pointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  waypoints->clear();
  pcl::PointXYZ point;
  int val1, val2, val3;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(waypoint_file, "%f", &point.x);
    val2 = fscanf(waypoint_file, "%f", &point.y);
    val3 = fscanf(waypoint_file, "%f", &point.z);

    if (val1 != 1 || val2 != 1 || val3 != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    waypoints->push_back(point);
  }

  fclose(waypoint_file);
}

void generate_waypoints()
{
  waypoints->clear();

  // Seed the random number generator
  srand(time(0));
  // Trees starting waypoints
  pcl::PointCloud<pcl::PointXYZ>::Ptr start_points(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::PointXYZ start_point_1 = {10.0, 15.0, 5.5};
  start_points->push_back(start_point_1);

  pcl::PointXYZ start_point_2 = {10.0, 0.0, 5.5};
  start_points->push_back(start_point_2);

  pcl::PointXYZ start_point_3 = {10.0, -30.0, 5.5}; //35
  start_points->push_back(start_point_3);

  int start_random_index = std::rand() % 3;
  waypoints->push_back(start_points->at(start_random_index));

  // pillars starting waypoints
  pcl::PointCloud<pcl::PointXYZ>::Ptr finish_points(new pcl::PointCloud<pcl::PointXYZ>());

  //pcl::PointXYZ finish_point_1 = {325.0, -10.0, 10.0};
  pcl::PointXYZ finish_point_1 = {1050.0, -15.0, 1.0};
  finish_points->push_back(finish_point_1);

  //pcl::PointXYZ finish_point_2 = {325.0, 14.0, 0.0 };
  pcl::PointXYZ finish_point_2 = {1050.0, 15.0, 1.0 };
  finish_points->push_back(finish_point_2);

  //pcl::PointXYZ finish_point_3 = {339.0, -22.0, 15.0 };
  pcl::PointXYZ finish_point_3 = {1050.0, 0.0, 1.0 };
  finish_points->push_back(finish_point_3);

  int finish_random_index = std::rand() % 3;
  waypoints->push_back(finish_points->at(finish_random_index));

}

// vehicle pose callback function
void poseHandler(const nav_msgs::Odometry::ConstPtr& pose)
{
  curTime = pose->header.stamp.toSec();

  vehicleX = pose->pose.pose.position.x;
  vehicleY = pose->pose.pose.position.y;
  vehicleZ = pose->pose.pose.position.z;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypointExample");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  
  // gets all the parameters defined in the launch file
  nhPrivate.getParam("waypoint_file_dir", waypoint_file_dir);   // waypoint filepath
  nhPrivate.getParam("waypointXYRadius", waypointXYRadius);     // how close to the waypoint to get in the XY plane   
  nhPrivate.getParam("waypointZBound", waypointZBound);         // how close to the waypoint to get in the Z plane
  nhPrivate.getParam("waitTime", waitTime);                     // how long to wait at the waypoint
  nhPrivate.getParam("frameRate", frameRate);                   // how often to publish the waypoint
  nhPrivate.getParam("speed", speed);                           // speed of the vehicle
  nhPrivate.getParam("sendSpeed", sendSpeed);                   // whether to send the speed to the vehicle

  // subscription to /state_estimation topic 
  ros::Subscriber subPose = nh.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, poseHandler);

  ros::Publisher pubWaypoint = nh.advertise<geometry_msgs::PointStamped> ("/way_point", 5);
  geometry_msgs::PointStamped waypointMsgs;
  waypointMsgs.header.frame_id = "map";

  ros::Publisher pubSpeed = nh.advertise<std_msgs::Float32> ("/speed", 5);
  std_msgs::Float32 speedMsgs;

  // read waypoints from file into waypoints
  //readWaypointFile();

  // Generate waypoints for sim environment 
  generate_waypoints();

  int wayPointID = 0;
  int waypointSize = waypoints->points.size();

  if (waypointSize == 0) {
    printf ("\nNo waypoint available, exit.\n\n");
    exit(1);
  }

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    float disX = vehicleX - waypoints->points[wayPointID].x;
    float disY = vehicleY - waypoints->points[wayPointID].y;
    float disZ = vehicleZ - waypoints->points[wayPointID].z;

    // start waiting if the current waypoint is reached
    if (sqrt(disX * disX + disY * disY) < waypointXYRadius && fabs(disZ) < waypointZBound && !isWaiting) {
      waitTimeStart = curTime;
      isWaiting = true;
    }

    // move to the next waypoint after waiting is over
    if (isWaiting && waitTimeStart + waitTime < curTime && wayPointID < waypointSize - 1) {
      wayPointID++;
      isWaiting = false;
    }

    // publish waypoint and speed messages at certain frame rate
    if (curTime - waypointTime > 1.0 / frameRate) {
      if (!isWaiting) {
        waypointMsgs.header.stamp = ros::Time().fromSec(curTime);
        waypointMsgs.point.x = waypoints->points[wayPointID].x;
        waypointMsgs.point.y = waypoints->points[wayPointID].y;
        waypointMsgs.point.z = waypoints->points[wayPointID].z;
        pubWaypoint.publish(waypointMsgs);
      }

      if (sendSpeed) {
        speedMsgs.data = speed;
        pubSpeed.publish(speedMsgs);
      }

      waypointTime = curTime;
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
