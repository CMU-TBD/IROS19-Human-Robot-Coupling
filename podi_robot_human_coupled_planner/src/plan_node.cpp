/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Bhaskara Marthi
 *         David V. Lu!!
 *********************************************************************/
#include <boost/shared_ptr.hpp>

#include <ros/console.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <podi_robot_human_coupled_planner/planner_core.h>
#include <podi_navigation_helpers/PodiMakeNavPlan.h>

using std::vector;
using geometry_msgs::PoseStamped;
using std::string;
using costmap_2d::Costmap2D;
using costmap_2d::Costmap2DROS;

namespace podi_robot_human_coupled_planner {

class PlannerWithCostmap : public GlobalPlanner {
    public:
        PlannerWithCostmap(string name, Costmap2DROS* cmap);
        bool makePlanService(podi_navigation_helpers::PodiMakeNavPlan::Request& req, podi_navigation_helpers::PodiMakeNavPlan::Response& resp);
    private:
        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);
        Costmap2DROS* cmap_;
        ros::ServiceServer make_plan_service_;
        ros::Subscriber pose_sub_;
};

bool PlannerWithCostmap::makePlanService(podi_navigation_helpers::PodiMakeNavPlan::Request& req, podi_navigation_helpers::PodiMakeNavPlan::Response& resp) {
    vector<PoseStamped> robotPlan;
    ROS_DEBUG("In PlannerWithCostmap::makePlanService");
    req.start.header.frame_id = "/map";
    req.goals.header.frame_id = "/map";
    if (req.goals.poses.size() <= 0) {
      ROS_ERROR("Called global planner make plan service with 0 goals.");
      return false;
    }

    tf::Stamped<tf::Pose> global_pose;
    if(!cmap_->getRobotPose(global_pose)) {
      ROS_ERROR("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    ROS_DEBUG("In PlannerWithCostmap::makePlanService 1, robot_frame=%s x=%f, y=%f, theta=%f", global_pose.frame_id_.c_str(), global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
    ROS_DEBUG("In PlannerWithCostmap::makePlanService 2, global_frame=%s, robot_frame=%s origin_x=%f, origin_y=%f, size_x=%d, size_y=%d, res=%f", cmap_->getGlobalFrameID().c_str(), cmap_->getBaseFrameID().c_str(), cmap_->getCostmap()->getOriginX(), cmap_->getCostmap()->getOriginY(), cmap_->getCostmap()->getSizeInCellsX(), cmap_->getCostmap()->getSizeInCellsY(), cmap_->getCostmap()->getResolution());

    bool success = makePlan(req.start, req.goals, robotPlan);
    resp.plan_found = success;
    if (success) {
        resp.path = robotPlan;
    }

    return true;
}

void PlannerWithCostmap::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal) {
    ROS_DEBUG("In PlannerWithCostmap::poseCallback");
    tf::Stamped<tf::Pose> global_pose;
    cmap_->getRobotPose(global_pose);
    vector<PoseStamped> robotPlan;
    geometry_msgs::PoseStamped start;
    start.header.stamp = global_pose.stamp_;
    start.header.frame_id = global_pose.frame_id_;
    start.pose.position.x = global_pose.getOrigin().x();
    start.pose.position.y = global_pose.getOrigin().y();
    start.pose.position.z = global_pose.getOrigin().z();
    start.pose.orientation.x = global_pose.getRotation().x();
    start.pose.orientation.y = global_pose.getRotation().y();
    start.pose.orientation.z = global_pose.getRotation().z();
    start.pose.orientation.w = global_pose.getRotation().w();
    geometry_msgs::PoseArray goals;
    goals.header = (*goal).header;
    goals.poses = std::vector<geometry_msgs::Pose>{(*goal).pose};
    makePlan(start, goals, robotPlan);
}

PlannerWithCostmap::PlannerWithCostmap(string name, Costmap2DROS* cmap) :
        GlobalPlanner(name, cmap) { // TODO (amal): eventually add a human footprint here!!!
    ros::NodeHandle private_nh("~");
    cmap_ = cmap;
    make_plan_service_ = private_nh.advertiseService("make_plan", &PlannerWithCostmap::makePlanService, this);
    pose_sub_ = private_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &PlannerWithCostmap::poseCallback, this);
}

} // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "podi_robot_human_coupled_planner");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
       ros::console::notifyLoggerLevelsChanged();
    }

    tf::TransformListener tf(ros::Duration(10));

    costmap_2d::Costmap2DROS lcr("costmap", tf);
    lcr.start();

    podi_robot_human_coupled_planner::PlannerWithCostmap pppp("planner", &lcr);

    ros::spin();
    return 0;
}
