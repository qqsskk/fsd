/*
 * AMZ-Driverless
 * Copyright (c) 2018 Authors:
 *   - Juraj Kabzan <kabzanj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// ROS Includes
#include "ros/ros.h"

// TF
#include "tf/transform_broadcaster.h"

#include "interface.hpp"

#include "nav_msgs/Odometry.h"
#include <fs_msgs/Cones.h>


// FSSIM
static ros::Subscriber sub_fssim_odom;
static ros::Subscriber sub_fssim_res;
static ros::Subscriber sub_fssim_track;

static ros::Publisher pub_fssim_cmd;
static ros::Publisher pub_fssim_mission_finnished;

// FSD_CAR
static ros::Subscriber sub_fsd_car_command;
static ros::Subscriber sub_fsd_mission_finnished;

static ros::Publisher pub_fsd_vel;
static ros::Publisher pub_fsd_state;
static ros::Publisher pub_fsd_map;
static ros::Publisher pub_gotthard_res;
static ros::Publisher pub_raw_odom_;

static bool                     tf_base_link = false;
static bool                     tf_odom_frame = false;
static std::string              fsd_vehicle;
static std::string              origin;
static std::string              odom_frame;
static std::string              odom_child_frame;
static tf::TransformBroadcaster *br = nullptr;

ros::Time current_time, last_time;
double x = 0.0;
double y = 0.0;
double th = 0.0;

void callbackFssimOdom(const fssim_common::State::ConstPtr &msg) {
    pub_fsd_vel.publish(gotthard::getStateDt(*msg));
    pub_fsd_state.publish(gotthard::getState(*msg));
    
    if (tf_base_link) {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, msg->yaw);
        transform.setRotation(q);
        br->sendTransform(tf::StampedTransform(transform, msg->header.stamp, origin, fsd_vehicle));
    }

    fsd_common_msgs::CarState state = gotthard::getState(*msg);
    fsd_common_msgs::CarStateDt state_dt = gotthard::getStateDt(*msg);

    double vx = state_dt.car_state_dt.x;
    double vy = state_dt.car_state_dt.y;
    double vth = state_dt.car_state_dt.theta;

    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    if (tf_odom_frame) {
        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = odom_frame;
        odom_trans.child_frame_id = fsd_vehicle;

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        br->sendTransform(odom_trans);
    }

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame;

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = fsd_vehicle;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    pub_raw_odom_.publish(odom);

    last_time = current_time;
}

void callbackFssimTrack(const fssim_common::Track::ConstPtr &msg) {
    pub_fsd_map.publish(gotthard::getMap(*msg));
}

void callbackFsdCmd(const fsd_common_msgs::ControlCommand::ConstPtr &msg) {
    pub_fssim_cmd.publish(fssim::getFssimCmd(*msg));
}

void callbackFsdMissionFinnished(const fsd_common_msgs::Mission::ConstPtr &msg) {
    pub_fssim_mission_finnished.publish(fssim::getFssimMissionFinnished(*msg));
}

template<class Type>
Type getParam(const ros::NodeHandle &nh, const std::string &name) {
    Type       val;
    const bool success = nh.getParam(name, val);
    assert(success && "PARAMETER DOES NOT EXIST");
    return val;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fssim_interface");
    ros::NodeHandle n("~");

    br = new tf::TransformBroadcaster();

    sub_fsd_car_command = n.subscribe(getParam<std::string>(n, "fsd/cmd"), 1, callbackFsdCmd);
    sub_fsd_mission_finnished =
        n.subscribe(getParam<std::string>(n, "fsd/mission_finnished"), 1, callbackFsdMissionFinnished);

    pub_fsd_vel   = n.advertise<fsd_common_msgs::CarStateDt>(getParam<std::string>(n, "fsd/vel"), 1);
    pub_fsd_state = n.advertise<fsd_common_msgs::CarState>(getParam<std::string>(n, "fsd/state"), 1);
    pub_fsd_map   = n.advertise<fsd_common_msgs::Map>(getParam<std::string>(n, "fsd/map"), 1, true);

    last_time = ros::Time::now();
    current_time = ros::Time::now();

    pub_raw_odom_ = n.advertise<nav_msgs::Odometry>("/odom", 1);

    sub_fssim_odom  = n.subscribe(getParam<std::string>(n, "fssim/topic_odom"), 1, callbackFssimOdom);
    sub_fssim_track = n.subscribe(getParam<std::string>(n, "fssim/track"), 1, callbackFssimTrack);

    pub_fssim_cmd = n.advertise<fssim_common::Cmd>(getParam<std::string>(n, "fssim/cmd"), 1);
    pub_fssim_mission_finnished =
        n.advertise<fssim_common::Mission>(getParam<std::string>(n, "fssim/mission_finnished"), 1);

    tf_base_link  = getParam<bool>(n, "fsd/tf/publish_car_base_link");
    tf_odom_frame = getParam<bool>(n, "fsd/tf/tf_odom_frame");
    origin        = getParam<std::string>(n, "fsd/tf/origin");
    fsd_vehicle   = getParam<std::string>(n, "fsd/tf/fsd_car_base_link");
    odom_frame        = getParam<std::string>(n, "fsd/tf/odom_frame");
    odom_child_frame  = getParam<std::string>(n, "fsd/tf/odom_child_frame");

    ros::spin();
    return 0;
}
