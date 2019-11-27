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
#include <string>

#include <ros/console.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>

#include <podi_robot_human_coupled_planner/planner_core.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(podi_robot_human_coupled_planner::GlobalPlanner, podi_nav_core::BaseGlobalPlanner)

namespace podi_robot_human_coupled_planner {

GlobalPlanner::~GlobalPlanner() {
    // NOTE (amal): I don't have to delete allocated Nodes here, since they will
    // be deleted by the planner_.
    if (planner_)
        delete planner_;
    if (grid_)
        delete grid_;
    if (robot_)
        delete robot_;
    if (world_model_)
        delete world_model_;
    if (couplingModel_)
        delete couplingModel_;
}

GlobalPlanner::GlobalPlanner() :
  costmap_(NULL), costmap_ros_(NULL), initialized_(false) {

}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
  costmap_(NULL), costmap_ros_(NULL), initialized_(false) {
  initialize(name, costmap_ros, costmap_ros->getRobotFootprint());
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  ROS_WARN("In GlobalPlanner::initialize");
  initialize(name, costmap_ros, costmap_ros->getRobotFootprint());
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros,
  std::vector<geometry_msgs::Point> robotFootprint) {
  if (!initialized_) {
      private_nh = ros::NodeHandle("~/" + name);
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      world_model_ = new podi_navigation_helpers::CostmapModel(*costmap_);
      frame_id_ = costmap_ros->getGlobalFrameID();
      private_nh.param("multiplicativeFactor", multiplicativeFactor_, 100);
      private_nh.param("heuristicRange", heuristicRange_, 10);
      private_nh.param("heuristicRangeMultiplier", heuristicRangeMultiplier_, 10);
      private_nh.param("heuristicRangeRequiredForSuccess", heuristicRangeRequiredForSuccess_, 80);
      int intervalToPubPath;
      private_nh.param("intervalToPubPath", intervalToPubPath, -1);

      convert_offset_ = 0.5; // NOTE (amal): I don't know why this is necessary...
      private_nh.param("numPointsToSubsample", numPointsToSubsample_, 1);

      // Set Up The Planner
      planner_ = new Planner(multiplicativeFactor_, numPointsToSubsample_, heuristicRange_, heuristicRangeMultiplier_, heuristicRangeRequiredForSuccess_, intervalToPubPath, name);
      // Set Up The Grid
      int numDiscretizations = 0;
      if (!private_nh.getParam("numDiscretizations", numDiscretizations))
        ROS_ERROR("Unable to get numDiscretizations at nodehanlde %s", private_nh.getNamespace().c_str());
      std::vector<std::vector<unsigned int> > gridSizes;
      for (int i = 0; i < numDiscretizations; ++i) {
        std::vector<int> gridSize;
        if (!private_nh.getParam("gridSize"+std::to_string(i), gridSize))
          ROS_ERROR("Unable to get param at index %d", i);
        std::vector<unsigned int> gridSizeFinal;
        for (int k = 0; k < gridSize.size(); ++k) gridSizeFinal.push_back((unsigned int)gridSize[k]);
        gridSizes.push_back(gridSizeFinal);
      }
      std::vector<int> distanceThresholds;
      private_nh.getParam("distanceThresholds", distanceThresholds);
      std::vector<unsigned int> distanceThresholdsFinal;
      for (int k = 0; k < distanceThresholds.size(); ++k) distanceThresholdsFinal.push_back((unsigned int)distanceThresholds[k]);
      std::vector<int> gridOrigin;
      private_nh.getParam("gridOrigin", gridOrigin);
      grid_ = new podi_navigation_helpers::Grid(gridSizes, distanceThresholdsFinal, gridOrigin);
      // Set Up The Robot
      int handleLength, maxV, minV, maxW, minW, minInPlaceW, maxLinAcc, maxRotAcc, linSimGranularity, rotSimGranularity;
      double simTime;
      private_nh.param("handle_length", handleLength, 43); // cm
      private_nh.param("max_vel_x", maxV, 60); // cm/s
      private_nh.param("min_vel_x", minV, 10); // cm/s
      private_nh.param("max_vel_theta", maxW, 34); // degrees/s
      private_nh.param("min_vel_theta", minW, 34); // degrees/s
      private_nh.param("min_in_place_vel_theta", minInPlaceW, 6); // degrees/s
      private_nh.param("acc_lim_x", maxLinAcc, 50); // cm/s^2
      private_nh.param("acc_lim_theta", maxRotAcc, 60); // degree/s^2
      private_nh.param("sim_time", simTime, 2.0); // s
      private_nh.param("sim_granularity", linSimGranularity, 3); // cm
      private_nh.param("angular_sim_granularity", rotSimGranularity, 2); // cm
      robot_ = new podi_navigation_helpers::Robot(handleLength, robotFootprint, maxV, minV, maxW, minW,
        minInPlaceW, maxLinAcc, maxRotAcc, simTime, linSimGranularity, rotSimGranularity);
      // Set Up The Coupling Model
      int handleLengthForCouplingModel, regionOffset, regionSize, regionAngle;
      double alpha, beta, distanceEqualityThreshold, orientationEqualityThreshold;
      int multiplicativeFactor = 1, intervalsWhenDrawingCurves;
      private_nh.param("handleLengthForCouplingModel", handleLengthForCouplingModel, 168); // mm
      private_nh.param("regionOffset", regionOffset, 505); // mm
      private_nh.param("regionSize", regionSize, 227); // mm
      private_nh.param("regionAngle", regionAngle, 57); // degrees
      private_nh.param("alpha", alpha, double(0.833333));
      private_nh.param("beta", beta, double(0.713992));
      private_nh.param("distanceEqualityThreshold", distanceEqualityThreshold, double(10.0));
      private_nh.param("orientationEqualityThreshold", orientationEqualityThreshold, double(1.0));
      private_nh.param("intervals_when_drawing_curves", intervalsWhenDrawingCurves, 10);
      couplingModel_ = new podi_navigation_helpers::CouplingModel(
        handleLengthForCouplingModel, regionOffset, regionSize, regionAngle, alpha, beta, distanceEqualityThreshold, orientationEqualityThreshold,
        intervalsWhenDrawingCurves, multiplicativeFactor_);

      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
      end_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("endGoal", 1);
      grid_test_pub_ = private_nh.advertise<visualization_msgs::Marker>("gridTest", 1);
      human_footprint_pub_ = private_nh.advertise<geometry_msgs::PolygonStamped>("humanFootprint", 1);

      // Get the params for the cost function
      private_nh.param("movementCost", movementCost_, 0.05);
      private_nh.param("rotationCost", rotationCost_, 2.5);
      private_nh.param("obsCost", obsCost_, 0.05);

      // Get the humanFootprint
      int humanFootprintNumPoints;
      humanFootprint_.clear();
      private_nh.param("humanFootprintNumPoints", humanFootprintNumPoints, 0);
      for (int i = 0; i < humanFootprintNumPoints; ++i) {
        geometry_msgs::Point pointMsg;
        std::vector<float> point;
        if (!private_nh.getParam("humanFootprintPoint"+std::to_string(i), point))
          ROS_ERROR("Unable to get param humanFootprintPoint%d", i);
        pointMsg.x = point[0];
        pointMsg.y = point[1];
        pointMsg.z = 0;
        humanFootprint_.push_back(pointMsg);
      }

      //get the tf prefix
      ros::NodeHandle nh;
      tf_prefix_ = tf::getPrefixParam(nh);

      make_plan_srv_ = private_nh.advertiseService("make_plan", &GlobalPlanner::makePlanService, this);

      initialized_ = true;

  } else
      ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
}

void GlobalPlanner::clearRobotCell(unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }
    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool GlobalPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    geometry_msgs::PoseArray goals;
    goals.header = req.goal.header;
    goals.poses = std::vector<geometry_msgs::Pose>{req.goal.pose};
    makePlan(req.start, goals, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}

void GlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool GlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseArray& goalsInit,
    std::vector<geometry_msgs::PoseStamped>& robotPlan) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    ROS_INFO("Robot_human_coupled_planner makePlan called!");

    //clear the plan, just in case
    robotPlan.clear();

    // Require the start pose and goal poses to be in the global_frame
    std::string global_frame = frame_id_;
    if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        return false;
    }
    if (tf::resolve(tf_prefix_, goalsInit.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goalsInit.header.frame_id).c_str());
        return false;
    }

    // Get the start pose and goal positions in costmap units
    double startx = start.pose.position.x;
    double starty = start.pose.position.y;
    unsigned int start_x_i, start_y_i;
    if (!costmap_->worldToMap(startx, starty, start_x_i, start_y_i)) {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    ROS_INFO("Robot_human_coupled_planner got start position in map frame!");
    double goalx, goaly;
    unsigned int goal_x_i, goal_y_i, max_goal_x_i, max_goal_y_i, min_goal_x_i, min_goal_y_i;
    int goaltheta;
    std::vector<std::vector<int> > goals;
    for (int i = 0; i < goalsInit.poses.size(); ++i) {
      geometry_msgs::Pose goal = goalsInit.poses[i];
      goalx = goal.position.x;
      goaly = goal.position.y;

      if (!costmap_->worldToMap(goalx, goaly, goal_x_i, goal_y_i)) {
          ROS_WARN(
                  "Goal %d sent to the global planner is off the global costmap. Planning will always fail to this goal.", i);
          continue;
      }
      if (i == 0) {
        max_goal_x_i = min_goal_x_i = goal_x_i;
        max_goal_y_i = min_goal_y_i = goal_y_i;
      } else {
        max_goal_x_i = std::max(max_goal_x_i, goal_x_i);
        min_goal_x_i = std::min(min_goal_x_i, goal_x_i);
        max_goal_y_i = std::max(max_goal_y_i, goal_y_i);
        min_goal_y_i = std::min(min_goal_y_i, goal_y_i);
      }
      goaltheta = radToDegree(tf::getYaw(goal.orientation));
      goals.push_back(std::vector<int>{(int)std::round(goalx*multiplicativeFactor_), (int)std::round(goaly*multiplicativeFactor_), (int)std::round(goaltheta)});
    }
    ROS_INFO("Robot_human_coupled_planner filtered through goals!");


    if (goals.size() == 0) {
      ROS_WARN("All goals are off the costmap. Planning will always fail.");
      return false;
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(start_x_i, start_y_i);

    // Create Nodes
    int robotx = startx*(multiplicativeFactor_);
    int roboty = starty*(multiplicativeFactor_);
    int robottheta = radToDegree(tf::getYaw(start.pose.orientation));
    // std::tuple<int, int, int> handlePose = robot_->getHandlePosition(robotx, roboty, robottheta);
    int v = 0, w = 0;
    grid_->setGoals(goals);
    const podi_navigation_helpers::Grid* gridPtr = const_cast<const podi_navigation_helpers::Grid*>(grid_);
    const Node* startNode = const_cast<const Node*>(new Node(
      world_model_, gridPtr, const_cast<const podi_navigation_helpers::Robot*>(robot_),
      const_cast<const podi_navigation_helpers::CouplingModel*>(couplingModel_),
      robotx, roboty, robottheta, v, w, multiplicativeFactor_, movementCost_,
      rotationCost_, obsCost_, humanFootprint_));
    ROS_INFO("Robot_human_coupled_planner created start node!");
    std::vector<const Node*> goalNodes;
    for (int i = 0; i < goals.size(); ++i) {
      const Node* goalNode = const_cast<const Node*>(new Node(
        gridPtr, goals[i][0], goals[i][1], goals[i][2], 0, 0));

      goalNodes.push_back(goalNode);
      ROS_INFO("Goal: %s", goalNode->id_.c_str());
    }

    // Visualize the Grid (TEMP)
    visualization_msgs::Marker gridTestViz;
    gridTestViz.type = visualization_msgs::Marker::SPHERE_LIST;
    gridTestViz.header.stamp = ros::Time::now();
    gridTestViz.header.frame_id = "/map";
    gridTestViz.action = visualization_msgs::Marker::ADD;
    gridTestViz.scale.x = 0.03;
    gridTestViz.scale.y = 0.03;
    gridTestViz.scale.z = 0.03;
    gridTestViz.pose.position.x = 0.0;
    gridTestViz.pose.position.y = 0.0;
    gridTestViz.pose.position.z = 0.0;
    gridTestViz.pose.orientation.x = 0.0;
    gridTestViz.pose.orientation.y = 0.0;
    gridTestViz.pose.orientation.z = 0.0;
    gridTestViz.pose.orientation.w = 1.0;
    gridTestViz.lifetime = ros::Duration(0.0);
    std::vector<geometry_msgs::Point> lethal, inscribed, offmap, noinfo, zero, pos, thetaLines;
    for (int x = std::min(start_x_i, min_goal_x_i); x <= std::max(start_x_i, max_goal_x_i); x = x + 1) {
      for (int y = std::min(start_y_i, min_goal_y_i); y <= std::max(start_y_i, max_goal_y_i); y = y + 1) {
        // if (x == 1050 && y == 200) {
        //   for (v = -360; v <= 360; ++v) {
        //     std::tuple<std::vector<int>, std::vector<unsigned int> > repElem = grid_->getGridCell(std::vector<int>{0, 0, 0, v, v});
        //     int repV = std::get<0>(repElem)[3];
        //     int repW = std::get<0>(repElem)[4];
        //     ROS_INFO("v=%d, w=%d, repV=%d, repW=%d", v, v, repV, repW);
        //   }
        // }
        double wx, wy;
        mapToWorld(x, y, wx, wy);
        int gridx = wx*multiplicativeFactor_, gridy = wy*multiplicativeFactor_;
        std::tuple<std::vector<int>, std::vector<unsigned int> > repElem = grid_->getGridCell(std::vector<int>{gridx, gridy});
        int repX = std::get<0>(repElem)[0];
        int repY = std::get<0>(repElem)[1];
        double wRepX = (double)repX/(double)multiplicativeFactor_;
        double wRepY = (double)repY/(double)multiplicativeFactor_;
        // ROS_INFO("wRepX=%f, wRepY=%f", wRepX, wRepY);
        geometry_msgs::Point point;
        point.x = wRepX;
        point.y = wRepY;
        point.z = 0.0;
        // Get the footprint cost of this point, hardcoded with theta = 0,
        // and determine the color based on that
        double cost = world_model_->footprintCost(wRepX, wRepY, degreeToRad(0.0), humanFootprint_);
        if (cost == podi_navigation_helpers::LETHAL) {
          lethal.push_back(point);
        } else if (cost == podi_navigation_helpers::INSCRIBED) {
          inscribed.push_back(point);
        } else if (cost == podi_navigation_helpers::OFFMAP) {
          offmap.push_back(point);
        } else if (cost == podi_navigation_helpers::NOINFO) {
          noinfo.push_back(point);
        } else if (cost == 0.0) {
          zero.push_back(point);
        } else {
          pos.push_back(point);
        }

        // Add the Theta Visualization
        double dist = 0.1;
        if (x % 60 == 0 && y % 60 == 0) {
          for (int theta = 0; theta < 360; theta = theta + 1) {
            repElem = grid_->getGridCell(std::vector<int>{gridx, gridy, theta});
            int repTheta = std::get<0>(repElem)[2];
            thetaLines.push_back(point);
            geometry_msgs::Point newPoint;
            newPoint.x = wRepX + dist*cos(degreeToRad(repTheta));
            newPoint.y = wRepY + dist*sin(degreeToRad(repTheta));
            newPoint.z = 0.0;
            thetaLines.push_back(newPoint);
          }
        }

      }
    }
    gridTestViz.id = 0;
    gridTestViz.color.r = 1.0;
    gridTestViz.color.g = 0.0;
    gridTestViz.color.b = 0.0;
    gridTestViz.color.a = 0.2;
    gridTestViz.points = pos;
    for (int foo = 0; foo < 2; ++foo) {
      grid_test_pub_.publish(gridTestViz);
    }
    gridTestViz.id = 1;
    gridTestViz.color.r = 0.0;
    gridTestViz.color.g = 1.0;
    gridTestViz.color.b = 0.0;
    gridTestViz.color.a = 0.2;
    gridTestViz.points = lethal;
    for (int foo = 0; foo < 2; ++foo) {
      grid_test_pub_.publish(gridTestViz);
    }
    gridTestViz.id = 2;
    gridTestViz.color.r = 0.0;
    gridTestViz.color.g = 0.0;
    gridTestViz.color.b = 1.0;
    gridTestViz.color.a = 0.2;
    gridTestViz.points = inscribed;
    for (int foo = 0; foo < 2; ++foo) {
      grid_test_pub_.publish(gridTestViz);
    }
    gridTestViz.id = 3;
    gridTestViz.color.r = 1.0;
    gridTestViz.color.g = 1.0;
    gridTestViz.color.b = 0.0;
    gridTestViz.color.a = 0.2;
    gridTestViz.points = offmap;
    for (int foo = 0; foo < 2; ++foo) {
      grid_test_pub_.publish(gridTestViz);
    }
    gridTestViz.id = 4;
    gridTestViz.color.r = 0.0;
    gridTestViz.color.g = 1.0;
    gridTestViz.color.b = 1.0;
    gridTestViz.color.a = 0.2;
    gridTestViz.points = noinfo;
    for (int foo = 0; foo < 2; ++foo) {
      grid_test_pub_.publish(gridTestViz);
    }
    gridTestViz.id = 5;
    gridTestViz.color.r = 1.0;
    gridTestViz.color.g = 0.0;
    gridTestViz.color.b = 1.0;
    gridTestViz.color.a = 0.2;
    gridTestViz.points = zero;
    for (int foo = 0; foo < 2; ++foo) {
      grid_test_pub_.publish(gridTestViz);
    }
    gridTestViz.id = 6;
    gridTestViz.scale.x = 0.007;
    gridTestViz.type = visualization_msgs::Marker::LINE_LIST;
    gridTestViz.color.r = 0.0;
    gridTestViz.color.g = 0.0;
    gridTestViz.color.b = 0.0;
    gridTestViz.color.a = 0.8;
    gridTestViz.points = thetaLines;
    for (int foo = 0; foo < 2; ++foo) {
      grid_test_pub_.publish(gridTestViz);
    }

    // Publish the humanFootprint at the startPose
    std::vector<geometry_msgs::Point32> points;
    for (int i = 0; i < humanFootprint_.size(); ++i) {
      geometry_msgs::Point point = humanFootprint_[i];
      // ROS_ERROR("footprint point x=%f, y=%f", point.x, point.y);
      geometry_msgs::Point32 newPoint;
      newPoint.x = point.x*std::cos(degreeToRad(startNode->humanTheta_)) - point.y*std::sin(degreeToRad(startNode->humanTheta_)) + (float)(startNode->humanX_)/(float)multiplicativeFactor_;
      newPoint.y = point.x*std::sin(degreeToRad(startNode->humanTheta_)) + point.y*std::cos(degreeToRad(startNode->humanTheta_)) + (float)(startNode->humanY_)/(float)multiplicativeFactor_;
      newPoint.z = 0;
      // ROS_ERROR("newPoint x=%f, y=%f", newPoint.x, newPoint.y);
      points.push_back(newPoint);
    }
    geometry_msgs::PolygonStamped polygon;
    polygon.header.frame_id = "/map";
    polygon.header.stamp = ros::Time::now();
    polygon.polygon.points = points;
    for (int foo = 0; foo < 2; ++foo) {
      human_footprint_pub_.publish(polygon);
    }

    // Ensure that the discretized robot pose is on the map and not on an obstacle
    float footprintCost = world_model_->footprintCost(
      (double)(startNode->robotX_)/(double)multiplicativeFactor_,
      (double)(startNode->robotY_)/(double)multiplicativeFactor_,
      degreeToRad(startNode->robotTheta_),
      robot_->footprint_
    );
    if (footprintCost < 0 && footprintCost != podi_navigation_helpers::INSCRIBED) {
      ROS_WARN("Robot start position collides with an obstacle or is off the map. Invalid start position.");
      // Delete the allocated nodes
      deleteNodes(startNode, goalNodes);
      return false;
    }

    // Ensure that the discretized human pose is on the map and not on an obstacle
    footprintCost = world_model_->footprintCost(
      (double)(startNode->humanX_)/(double)multiplicativeFactor_,
      (double)(startNode->humanY_)/(double)multiplicativeFactor_,
      degreeToRad(startNode->humanTheta_),
      humanFootprint_
    );
    ROS_INFO("Human footprint cost = %f, footprintSize=%lu", footprintCost, humanFootprint_.size());
    if (footprintCost < 0 && footprintCost != podi_navigation_helpers::INSCRIBED) {
      ROS_WARN("Human start position collides with an obstacle or is off the map. Invalid start position.");
      // Delete the allocated nodes
      deleteNodes(startNode, goalNodes);
      return false;
    }

    // NOTE: maybe we want to allow goals that are off the map or collide with
    // obstacles, since we can still get close to them?
    // Remove any discretized goal poses that are on obstacles or off the map
    int i = 0;
    while (i < goalNodes.size()) {
      footprintCost = world_model_->footprintCost(
        (double)(goalNodes[i]->humanX_)/(double)multiplicativeFactor_,
        (double)(goalNodes[i]->humanY_)/(double)multiplicativeFactor_,
        degreeToRad(goalNodes[i]->humanTheta_),
        humanFootprint_
      );
      if (footprintCost < 0 && footprintCost != podi_navigation_helpers::INSCRIBED) {
        ROS_WARN("Goal %d collides with an obstacle or is off the map. Invalid start position.", i);
        // Delete the allocated goal node
        delete goalNodes[i];
        goalNodes.erase(goalNodes.begin() + i);
        // Keep i the same since the goalNodes vector has gotten smaller by one
      } else {
        i++; // increment i
      }
    }
    if (goalNodes.size() == 0) {
      ROS_WARN("All goal nodes either collide with an obstacle or are off the map. Invalid goal positions.");
      // Delete the allocated nodes
      deleteNodes(startNode, goalNodes);
      return false;
    }

    // Call the Planner

    private_nh.param("cycles", cycles_, -1);
    ROS_INFO("In GlobalPlanner::initialize cycles_ = %d", cycles_);

    std::vector<std::tuple<int, int, int> > robotPath;
    const Node* endGoal;
    bool found_legal = planner_->getPath(startNode, goalNodes, cycles_, robotPath, endGoal);

    if (found_legal) {
        //extract the plan
        if (!getPlanFromPath(robotPath, robotPlan)) {
            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }
    } else {
        ROS_ERROR("Failed to get a plan.");
    }

    // Publish the plan for visualization purposes
    publishPlan(robotPlan, endGoal);
    planner_->clearDataStructures();

    // Delete the allocated nodes
    deleteNodes(startNode, goalNodes);

    return !robotPlan.empty();
}

void GlobalPlanner::deleteNodes(const Node* startNode, std::vector<const Node*> goalNodes) {
  delete startNode;
  for (int i = 0; i < goalNodes.size(); ++i) {
    delete goalNodes[i];
  }
}

void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const Node* endGoal) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    geometry_msgs::PoseStamped goal;

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
        if (i == path.size() - 1) {
          goal.header = path[i].header;
        }
    }

    goal.pose.position.x = (double)(endGoal->robotX_)/(double)multiplicativeFactor_;
    goal.pose.position.y = (double)(endGoal->robotY_)/(double)multiplicativeFactor_;
    goal.pose.position.z = 0.0;
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(degreeToRad(endGoal->robotTheta_));

    plan_pub_.publish(gui_path);
    end_goal_pub_.publish(goal);
}

bool GlobalPlanner::getPlanFromPath(std::vector<std::tuple<int, int, int> >& robotPath,
                                             std::vector<geometry_msgs::PoseStamped>& robotPlan) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = frame_id_;

    //clear the plan, just in case
    robotPlan.clear();

    ros::Time plan_time = ros::Time::now();
    ROS_INFO("ROBOT PLAN:");
    for (int i = robotPath.size() - 1; i>=0; --i) { // Go backwards to reverse the path
        std::tuple<int, int, int> xyTheta = robotPath[i];
        //convert the plan to world coordinates
        double x = ((double)std::get<0>(xyTheta))/(double)(multiplicativeFactor_);
        double y = ((double)std::get<1>(xyTheta))/(double)(multiplicativeFactor_);
        double theta = degreeToRad(std::get<2>(xyTheta));
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
        ROS_INFO("x=%f, y=%f, theta=%f", x, y, theta);
        robotPlan.push_back(pose);
    }
    return !robotPlan.empty();
}

} //end namespace podi_robot_human_coupled_planner
