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
#include "cone_detector.hpp"
#include <sstream>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <fs_msgs/Cones.h>
#include <fs_msgs/Cone.h>

// typedef pcl::PointCloud<pcl::PointXYZ> CloudType;


namespace ns_cone_detector {
    // Constructor
    ConeDetector::ConeDetector() {
    };

    // Getters
    fsd_common_msgs::ConeDetections ConeDetector::getConeDetections() const { return coneDetections_; }
    fs_msgs::Cones ConeDetector::getConeDetectionsMsg() const { return cones_detected_msg_; }

    void ConeDetector::runAlgorithm(const sensor_msgs::PointCloud2 &msg, const fssim_common::State &state) {
      ROS_INFO("Point cloud message received" );
      
      // sensor_msgs::PointCloud2 raw_cloud_temp = msg;
      // bool result;
      // result = pcl::concatenatePointCloud(current_cloud_, raw_cloud_temp, current_cloud_);
      // if (result) {
      //     concatination_count_++;
      // } else {
      //     ROS_INFO("Pointcloud dropped as concatenation failed");
      // }

      // if (concatination_count_ < 5)
      //   return;  
      // ROS_INFO("Pointcloud BUILT");
	    // ROS_INFO("Loaded pointcloud with %lu points.", filtered_cloud->size());

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

      pcl::fromROSMsg(msg, *cloud_filtered);
      std::cerr << "PointCloud SIZE: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

      // Write the downsampled version to disk
      // pcl::PCDWriter writer;
      // writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (1000);
      seg.setDistanceThreshold (0.01);

      // Create the filtering object
      pcl::ExtractIndices<pcl::PointXYZ> extract;

      int i = 0, nr_points = (int) cloud_filtered->points.size ();
      // While 30% of the original cloud is still there
      while (cloud_filtered->points.size () > 0.3 * nr_points)
      {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
          std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
          break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        // std::stringstream ss;
        // ss << "table_scene_lms400_plane_" << i << ".pcd";
        // writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_filtered.swap (cloud_f);
        i++;
      }

      // ROS_INFO("DONE DONE DONE DONE DONE DONE DONE");

      // Our cones message
      fs_msgs::Cones cones_msg;   
      cones_msg.header.frame_id = msg.header.frame_id;
      cones_msg.header.stamp = ros::Time::now();
      for (auto p: cloud_p->points) {
        // ROS_INFO("x: %f, y: %f, z: %f", p.x, p.y, p.z);
        fs_msgs::Cone cone;
        cone.x = p.x;
        cone.y = p.y;
        cone.color = cone.UNDEFINED;
        // TODO: how to decide ?
        // 
        if(p.y > 0){
          cone.color = ("_left_color" == "yellow") ? cone.BLUE : cone.YELLOW;
        } else{
          cone.color = ("_right_color" == "yellow") ? cone.YELLOW : cone.BLUE;
        }
        cone.covariance[0] = 0.005;
        cone.covariance[1] = 0;
        cone.covariance[2] = 0;
        cone.covariance[3] = 0.005;
        cones_msg.cones.push_back(cone);
      }
      cones_detected_msg_ = cones_msg; 

      std::vector<fsd_common_msgs::Cone> cones;
      fsd_common_msgs::Cone cone;
      // Create 3 random yellow cones
      for (auto p: cloud_p->points) {
        cone.position.x = p.x;
        cone.position.y = p.y;
        cone.color.data = (p.y > 0) ? "y" : "b";
        cones.push_back(cone);
      }
      coneDetections_.cone_detections = cones;

      // createConeDetections();
    }

    void ConeDetector::createConeDetections() {
      std::vector<fsd_common_msgs::Cone> cones;
      fsd_common_msgs::Cone cone;
      // Create 3 random yellow cones
      for (int i = 0; i < 3; i++)
      {
        cone.position.x = 15 * ((double) rand() / (RAND_MAX));
        cone.position.y = 15 * ((double) rand() / (RAND_MAX));
        cone.color.data = "y";
        cones.push_back(cone);
      }
      coneDetections_.cone_detections = cones;
    }

}
