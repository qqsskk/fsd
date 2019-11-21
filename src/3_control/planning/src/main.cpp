#include <ros/ros.h>
#include "planning_handle.hpp"

#include "objects/Cone.hpp"
#include "objects/Line.hpp"
#include "TrackBoundaryEstimator.hpp"
#include <vector>

#include "fs_msgs/Cones.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PolygonStamped.h"

// typedef ns_planning::PlanningHandle PlanningHandle;

static ros::Subscriber cones_sub_;
static ros::Publisher blue_line_pub_;
static ros::Publisher yellow_line_pub_;

int connectionType = TrackBoundaryEstimator::CONNECTION_FULL; //Used for first lap of autocross
// int connectionType = TrackBoundaryEstimator::CONNECTION_SNAKE; //Used for laps 1-9 of autoross
// int connectionType = TrackBoundaryEstimator::CONNECTION_ACCELERATION; //used for all of acceleration
// int connectionType = TrackBoundaryEstimator::CONNECTION_SKID_PAD; //Used for all of skid pad

TrackBoundary trackBoundary;

void publishLine(const std::vector<Line> &lines, const std::string &color)
{
  geometry_msgs::PolygonStamped center_line;
  center_line.polygon.points.clear();

  for (const auto &l: lines) {
    geometry_msgs::Point32 p;
    p.x = l.ax;
    p.y = l.ay;
    p.z = 0.0;
    center_line.polygon.points.push_back(p);
    p.x = l.bx;
    p.y = l.by;
    p.z = 0.0;
    center_line.polygon.points.push_back(p);
  }

  center_line.header.frame_id = "map";
  center_line.header.stamp    = ros::Time::now();
  if (color == "blue") { blue_line_pub_.publish(center_line); }
  else {yellow_line_pub_.publish(center_line);}
}

void callbackCones(const fs_msgs::Cones::ConstPtr &msg) {
  ROS_INFO("READ CONES");
  // std::vector<fs_msgs::Cone> cones = msg->cones;

  //Input cones data
  std::vector<Cone> blueCones = {};
  std::vector<Cone> yellowCones = {};
  std::vector<Cone> orangeCones = {};

  for (auto &cone: msg->cones)
  {
      // ROS_INFO("x: %f, y: %f, color: %d", cone.x, cone.y, cone.color);
      
      Cone c {cone.x, cone.y, cone.color};

      switch(cone.color) {
        case 1: { yellowCones.push_back(c); }
        case 2: { blueCones.push_back(c); }
        // case 3: { orangeCones.push_back(c); }
      }
  }

  ROS_INFO("blue cones count: %d", (int)blueCones.size());
  ROS_INFO("yellow cones count: %d", (int)yellowCones.size());

  // Connect track
  trackBoundary = TrackBoundaryEstimator::connect(connectionType, blueCones, yellowCones, orangeCones);
  
  // Get output
  std::vector<Line> blueLines =  trackBoundary.blueLines;
  std::vector<Line> yellowLines = trackBoundary.yellowLines;
  std::vector<Line> orangeLines = trackBoundary.orangeLines;

  publishLine(blueLines, "blue");
  publishLine(yellowLines, "yellow");

  ROS_INFO("blue lines count: %d", (int)blueLines.size());

}

// TODO: PID for control ??

int main(int argc, char **argv) {

  ros::init(argc, argv, "planning");
  ros::NodeHandle nodeHandle("planning");

  // Sub
  cones_sub_ = nodeHandle.subscribe("/cones", 1, callbackCones);

  // Pub
  blue_line_pub_ = nodeHandle.advertise<geometry_msgs::PolygonStamped>("/blue", 1);
  yellow_line_pub_ = nodeHandle.advertise<geometry_msgs::PolygonStamped>("/yellow", 1);

  // PlanningHandle planningHandle(nodeHandle);
  // ros::Rate loop_rate(planningHandle.getNodeRate());
  
  ros::Rate loop_rate(1);

  while (ros::ok()) {
    ROS_INFO("RUNNING!");
    ros::spinOnce();                
    loop_rate.sleep();          
  }
  
  return 0;
}

