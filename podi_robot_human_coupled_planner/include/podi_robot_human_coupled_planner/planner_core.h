#ifndef _PLANNERCORE_H
#define _PLANNERCORE_H
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
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/

#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <podi_nav_core/base_global_planner.h>

#include <podi_navigation_helpers/robot.h>
#include <podi_navigation_helpers/couplingModel.h>
#include <podi_navigation_helpers/grid.h>
#include <podi_navigation_helpers/world_model.h>
#include <podi_navigation_helpers/costmap_model.h>

#include <podi_robot_human_coupled_planner/planner.h>
#include <podi_robot_human_coupled_planner/node.h>

namespace podi_robot_human_coupled_planner {

/**
 * @class PlannerCore
 * @brief Provides a ROS wrapper for the podi_robot_human_coupled_planner planner which runs a fast, interpolated navigation function on a costmap.
 */

class GlobalPlanner : public podi_nav_core::BaseGlobalPlanner {
    public:

        /**
         * @brief  Constructor for the PlannerCore object
         * @param  name The name of this planner
         * @param  costmap A pointer to the costmap to use
         * @param  frame_id Frame of the costmap
         */
        GlobalPlanner();
        GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /**
         * @brief  Default deconstructor for the PlannerCore object
         */
        ~GlobalPlanner();

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros, std::vector<geometry_msgs::Point> robotFootprint);

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseArray& goals,
                      std::vector<geometry_msgs::PoseStamped>& robotPlan);

        /**
         * @brief Compute a plan to a goal after the potential for a start point has already been computed (Note: You should call computePotential first)
         * @param start_x
         * @param start_y
         * @param end_x
         * @param end_y
         * @param goal The goal pose to create a plan to
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool getPlanFromPath(std::vector<std::tuple<int, int, int> >& robotPath,
                             std::vector<geometry_msgs::PoseStamped>& robotPlan);

        /**
         * @brief  Publish a path for visualization purposes
         */
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const Node* endGoal);

        bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

    protected:

        /**
         * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
         */
        costmap_2d::Costmap2DROS* costmap_ros_;
        costmap_2d::Costmap2D* costmap_;
        podi_navigation_helpers::WorldModel* world_model_; ///< @brief The world model that the controller will use
        std::string frame_id_;
        ros::Publisher plan_pub_, grid_test_pub_, human_footprint_pub_, end_goal_pub_;
        bool initialized_/*, allow_unknown_, visualize_potential_*/;

    private:
        void mapToWorld(double mx, double my, double& wx, double& wy);
        bool worldToMap(double wx, double wy, double& mx, double& my);
        void clearRobotCell(unsigned int mx, unsigned int my);

        std::string tf_prefix_;
        boost::mutex mutex_;
        ros::ServiceServer make_plan_srv_;

        Planner* planner_;
        podi_navigation_helpers::Grid* grid_;
        podi_navigation_helpers::Robot* robot_;
        podi_navigation_helpers::CouplingModel* couplingModel_;

        int32_t cycles_;

        float degreeToRad(int degree) const {
          return degree*M_PI/180;
        };
        int radToDegree(float rad) const {
          return rad*180/M_PI;
        };

        int multiplicativeFactor_, numPointsToSubsample_, heuristicRange_, heuristicRangeMultiplier_, heuristicRangeRequiredForSuccess_;

        float convert_offset_;

        double movementCost_, rotationCost_, obsCost_;

        void deleteNodes(const Node* startNode, std::vector<const Node*> goalNodes);

        std::vector<geometry_msgs::Point> humanFootprint_;

        ros::NodeHandle private_nh;

};

} //end namespace podi_robot_human_coupled_planner

#endif
