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

#ifndef PERCEPTION_VISION_CONE_DETECTOR_HPP
#define PERCEPTION_VISION_CONE_DETECTOR_HPP

#include "fsd_common_msgs/ConeDetections.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"


namespace ns_cone_detector {

    class ConeDetector {

    public:
        // Constructor
        ConeDetector();

        // Getters
        fsd_common_msgs::ConeDetections getConeDetections() const;

        /**
         *  creates the cone detections
         */
        void createConeDetections(const sensor_msgs::PointCloud2 &camera_point_cloud_);

        /**
         * calls the other functions in the right order
         */
        void runAlgorithm(const sensor_msgs::PointCloud2 &camera_point_cloud_);

    private:
    
      fsd_common_msgs::ConeDetections coneDetections_;

    };
}

#endif //PERCEPTION_VISION_CONE_DETECTOR_HPP
