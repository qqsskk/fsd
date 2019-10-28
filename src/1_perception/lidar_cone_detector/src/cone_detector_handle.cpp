/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2018:
     - Sonja Brits <britss@ethz.ch>

    FSD-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FSD-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include "cone_detector_handle.hpp"

#include <fs_msgs/Cones.h>

namespace ns_cone_detector {

// Constructor
ConeDetectorHandle::ConeDetectorHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  subscribeToTopics();
  publishToTopics();
}

// Getters
int ConeDetectorHandle::getNodeRate() const { return node_rate_; }

// Methods
void ConeDetectorHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  if (!nodeHandle_.param<std::string>("cone_detections_topic_name", cone_detections_topic_name_, "/perception/cone_detections")) {
    ROS_WARN_STREAM("Did not load cone_detections_topic_name. Standard value is: " << cone_detections_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("cone_detections_topic_name_2", cone_detections_topic_name_2_, "/cones")) {
    ROS_WARN_STREAM("Did not load cone_detections_topic_name. Standard value is: " << cone_detections_topic_name_2_);
  }
  if (!nodeHandle_.param<std::string>("lidar_topic_name", lidar_topic_name_, "/lidar/cones")) {
    ROS_WARN_STREAM("Did not load lidar topic. Standard value is: " << lidar_topic_name_);
  }
  if (!nodeHandle_.param("node_rate", node_rate_, 1)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }
}

void ConeDetectorHandle::callbackRawLidar(const sensor_msgs::PointCloud2 &msg) {
  ROS_INFO("Lidar points callback");
  // lidar_point_cloud_ = msg;
  coneDetector_.runAlgorithm(msg);
}

void ConeDetectorHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  lidarSubscriber = nodeHandle_.subscribe(lidar_topic_name_, 1, &ConeDetectorHandle::callbackRawLidar, this);
}

void ConeDetectorHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  coneDetectionsPublisher = nodeHandle_.advertise<fsd_common_msgs::ConeDetections>(cone_detections_topic_name_, 1);
  coneDetectionsPublisher2 = nodeHandle_.advertise<fs_msgs::Cones>(cone_detections_topic_name_2_, 1);
}

void ConeDetectorHandle::run() {
  sendConeDetections();
}

void ConeDetectorHandle::sendConeDetections() {
  cone_detections_.cone_detections = coneDetector_.getConeDetections().cone_detections;
  cone_detections_.header.stamp = ros::Time::now();
  coneDetectionsPublisher.publish(cone_detections_);

  cones_msg_ = coneDetector_.getConeDetectionsMsg();
  cones_msg_.header.stamp = ros::Time::now();
  coneDetectionsPublisher2.publish(cones_msg_);
  ROS_INFO("cone detections sent");
}
}
