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
#include <cmath>
#include <set>
#include <algorithm>

#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <costmap_2d/cost_values.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

#include <podi_robot_human_coupled_planner/planner.h>

namespace podi_robot_human_coupled_planner {

Planner::Planner(int multiplicativeFactor, int numPointsToSubsample, int heuristicRange,
  int heuristicRangeMultiplier, int heuristicRangeRequiredForSuccess, int intervalToPubPath, std::string name) {
  ros::NodeHandle private_nh("~/" + name);
  // Create the publishers
  robot_traj_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("robot_traj_goal", 1);
  human_traj_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("human_traj_goal", 1);

  robot_plan_pub_ = private_nh.advertise<nav_msgs::Path>("robot_global_plan", 1);
  exact_robot_plan_pub_ = private_nh.advertise<nav_msgs::Path>("robot_exact_global_plan", 1);
  human_plan_pub_ = private_nh.advertise<nav_msgs::Path>("human_global_plan", 1);
  exact_human_plan_pub_ = private_nh.advertise<nav_msgs::Path>("human_exact_global_plan", 1);

  search_robot_plan_pub_ = private_nh.advertise<nav_msgs::Path>("robot_global_plan_during_search", 1);
  // Set the instance variables
  multiplicativeFactor_ = multiplicativeFactor;
  numPointsToSubsample_ = numPointsToSubsample;
  heuristicRange_ = heuristicRange;
  heuristicRangeMultiplier_ = heuristicRangeMultiplier;
  heuristicRangeRequiredForSuccess_ = heuristicRangeRequiredForSuccess;
  intervalToPubPath_ = intervalToPubPath;
}

Planner::~Planner() {
  const Node* currNode;
  std::set<const Node*> deletedNodes;
  // Delete all Nodes currently in the queue
  for (int i = 0; i < queue_.size(); i++) {
    MinHeapElem top = queue_[0];
    currNode = top.node;
    // Check that the element has not already been deleted
    std::set<const Node*>::iterator it = deletedNodes.find(currNode);
    if (it != deletedNodes.end() && currNode) {
      deletedNodes.insert(currNode);
      delete currNode;
    }
  }
  // Delete all Nodes that have been evaluated
  for (std::unordered_map<nodeIDType, const Node*>::iterator itr = evaluated_.begin(); itr != evaluated_.end(); ++itr) {
    currNode = itr->second;
    // Check that the element has not already been deleted
    std::set<const Node*>::iterator it = deletedNodes.find(currNode);
    if (it != deletedNodes.end() && currNode) {
      deletedNodes.insert(currNode);
      delete currNode;
    }
  }
}

void Planner::clearDataStructures(std::vector<const Node*> nodesToKeep) {
  const Node* currNode;
  std::set<const Node*> deletedNodes;
  // The way we keep nodes in nodesToKeep is by pretending that they were already deleted
  for (int i = 0; i < nodesToKeep.size(); ++i) {
    deletedNodes.insert(nodesToKeep[i]);
  }
  // Delete all Nodes currently in the queue
  for (int i = 0; i < queue_.size(); i++) {
    MinHeapElem top = queue_[0];
    currNode = top.node;
    // Check that the element has not already been deleted
    std::set<const Node*>::iterator it = deletedNodes.find(currNode);
    if (it != deletedNodes.end() && currNode) {
      deletedNodes.insert(currNode);
      delete currNode;
    }
  }
  queue_.clear();
  // Delete all Nodes that have been evaluated
  for (std::unordered_map<nodeIDType, const Node*>::iterator itr = evaluated_.begin(); itr != evaluated_.end(); ++itr) {
    currNode = itr->second;
    // Check that the element has not already been deleted
    std::set<const Node*>::iterator it = deletedNodes.find(currNode);
    if (it != deletedNodes.end() && currNode) {
      deletedNodes.insert(currNode);
      delete currNode;
    }
  }
  evaluated_.clear();
  costs_.clear();
  parents_.clear();
}

bool Planner::getPath(const Node* start, std::vector<const Node*> goalNodes, int cycles,
    std::vector<std::tuple<int, int, int> >& robotPath, const Node*& endGoal) {

    // Extra bookkeeping variables to understamnd the functionality of this
    // planner
    unsigned int nodesPoppedOffHeap = 0;
    unsigned int nodesPushedOntoHeap = 0;
    unsigned int totalNodesChecked = 0; // includes neighbors of nodes we popped off of the head, double counts nodes that were already evaluated
    unsigned int nodesThatWereAlreadyEvaluated = 0; // counts neighbors of nodes, where the neighbors were already evaluated. Can double count

    // // If the data structures have not been emptied, empty them
    // if (queue_.size() > 0 || evaluated_.size() > 0 || costs_.size() > 0 || parents_.size() > 0) {
    //   ROS_INFO("Planner::getPath clearing data structures");
    //   clearDataStructures();
    //   ROS_INFO("Planner::getPath cleared data structures");
    // }

    // Initialize datatypes with appropriate values
    double startHeuristic = start->heuristic(goalNodes);
    queue_.push_back(MinHeapElem(start, startHeuristic)); // Push onto the vector
    std::push_heap(queue_.begin(), queue_.end(), minHeapCmp()); // Re-order it based on heap properties
    nodesPushedOntoHeap++;
    totalNodesChecked++;
    parents_[start->id_] = nullptr;
    costs_[start->id_] = 0.0;

    // Since we may not actually reach the goal, we use ranges to get the best
    // path so far. Basically, for every path, we determine what fraction of the
    // path it has completed (by dividing its heuristic by the start heuristic).
    // We then multiply it by the multiplier. For that value, we then bucket it
    // into an interval of size heuristicRange_. Within the paths in the largest
    // interval we have seen so far, it takes the min cost path. This ensures
    // that we get paths that end up close to the goal, but that also minimize
    // cost.
    int minHeuristicRange = 0;
    bool isMinPathCostSet = false;
    double minPathCost;

    // Initialize variables used for the search
    int cycle = 0;
    const Node* currNode;
    std::unordered_map<nodeIDType, const Node*>::iterator searchEval;
    std::unordered_map<nodeIDType, double>::iterator currNodeCost, neighborCost;

    while (queue_.size() > 0 && ros::ok()) {
        if (cycles > -1 && cycle >= cycles) {
          ROS_INFO("TIMEOUT, reached %d cycles", cycle);
          break; // If cycles == -1, then no limit
        }

        // Get the minCost elem from the heap
        MinHeapElem top = queue_[0];
        std::pop_heap(queue_.begin(), queue_.end(), minHeapCmp()); // remove first elem
        queue_.pop_back(); // decrease the container size by one
        currNode = top.node;
        nodesPoppedOffHeap++;

        // ROS_ERROR("Cycle %d, intervalToPubPath=%d", cycle, intervalToPubPath);
        if (intervalToPubPath_ > -1 && (cycle % intervalToPubPath_) == 0) recreatePathSearch(start, currNode);

        // ROS_INFO("CurrNode = %s With Cost + Heuristic = %f", currNode->id_.c_str(), top.cost);

        // Skip nodes that have already been evaluated. This can happen if we
        // pushed node N to the queue, then found a shorter path to node N
        // before N was popped off of the queue, and added that.
        searchEval = evaluated_.find(currNode->id_);
        if (searchEval != evaluated_.end()) { // element found
          nodesThatWereAlreadyEvaluated++;
          delete currNode;
          continue;
        }

        // Insert the currNode into our evaluated_ map
        std::pair<std::unordered_map<nodeIDType, const Node*>::iterator, bool> retVal = evaluated_.insert(std::make_pair(currNode->id_, currNode)); // TODO: perhaps check the return value to see if the insert succeeded?
        if (!retVal.second) {
          ROS_ERROR("Failed to insert node %s into the evaluated_ map", currNode->id_.c_str());
        }

        // Check if currNode is a goal
        for (int k = 0; k < goalNodes.size(); ++k) {
          const Node* goal = goalNodes[k];
          if ((*currNode).reachedGoal(goal)) {
            ROS_INFO("FOUND GOAL!!!, CYCLE: %d", cycle);
            minHeuristicRange = ((int)round((1.0)*heuristicRangeMultiplier_))/heuristicRange_*heuristicRange_;
            isMinPathCostSet = true;
            currNodeCost = costs_.find(currNode->id_);
            if (currNodeCost == costs_.end()) { // element not found
              continue;
            }
            minPathCost = currNodeCost->second;
            recreatePath(start, currNode, robotPath);
            endGoal = goal;
            return true;
          }
        }

        // Get the cost of the currNode
        currNodeCost = costs_.find(currNode->id_);
        if (currNodeCost == costs_.end()) { // element not found
          ROS_ERROR("Failed to get cost of node %s  that was popped off the heap", currNode->id_.c_str());
          ROS_ERROR("This should not happen...");
          evaluated_.erase(currNode->id_);
          delete currNode;
          break;
        }

        // Get the heuristic
        const Node* goal;
        double heuristicDistance = (currNode->heuristic)(goalNodes, goal);
        // If this path is the best path we have seen so far (based on heuristic
        // range and the path cost), then take it.
        int currHeuristicRange = ((int)round((1.0-heuristicDistance/startHeuristic)*heuristicRangeMultiplier_))/heuristicRange_*heuristicRange_;
        if (currHeuristicRange > minHeuristicRange ||
           (currHeuristicRange == minHeuristicRange && (!isMinPathCostSet || currNodeCost->second < minPathCost))) {
          minHeuristicRange = currHeuristicRange;
          isMinPathCostSet = true;
          minPathCost = currNodeCost->second;
          ROS_INFO("Best Path So Far Has Range = %d, Cost = %f, CYCLE: %d", minHeuristicRange, minPathCost, cycle);
          recreatePath(start, currNode, robotPath);
          endGoal = goal;
          if (minHeuristicRange >= 95) break;
        }

        // Iterate over all currNode's neighbors
        const std::vector<std::tuple<const Node*, double> >& neighbors = (currNode->neighbors)();

        for (int i = 0; i < neighbors.size(); i++) {
          const Node* neighbor = std::get<0>(neighbors[i]);
          totalNodesChecked++;

          // Skip neighbors that have already been evaluated
          searchEval = evaluated_.find(neighbor->id_);
          if (searchEval != evaluated_.end()) { // element found
            nodesThatWereAlreadyEvaluated++;
            delete neighbor;
            continue;
          }

          // Get the actual cost of this path
          double pathCost = std::get<1>(neighbors[i]) + currNodeCost->second;

          // If either the neighbor is not in not in costs_, or it is but the
          // current path has a smaller cost, then add the node to queue
          neighborCost = costs_.find(neighbor->id_);

          if (neighborCost == costs_.end() || pathCost < neighborCost->second) {
            double neighborHeuristic = (neighbor->heuristic)(goalNodes);
            // ROS_INFO("Adding Neighbor = %s With Cost = %f And Heuristic = %f", neighbor->id_.c_str(), pathCost, neighborHeuristic);
            costs_.insert(std::make_pair(neighbor->id_, pathCost));
            queue_.push_back(MinHeapElem(neighbor, pathCost + neighborHeuristic));
            std::push_heap(queue_.begin(), queue_.end(), minHeapCmp());
            nodesPushedOntoHeap++;
            parents_.insert(std::make_pair(neighbor->id_, currNode));
          }
        }
        cycle++;
    }

    if (minHeuristicRange >= heuristicRangeRequiredForSuccess_) return true;
    return false;
}

float Planner::degreeToRad(int degree) const {
  return degree*M_PI/180;
}

bool Planner::recreatePath(const Node* start, const Node* currNode,
  std::vector<std::tuple<int, int, int> >& robotPath) {
  // Ensure that the robotPath is empty, and set up the paths we will publish
  robotPath.clear();
  std::vector<std::tuple<int, int, int> > exactRobotPath, exactHumanPath, humanPath;
  std::vector<geometry_msgs::Pose> humanTrajPoses, robotTrajPoses;
  // start from the currNode (which is close to the goal) and work backwards
  // to the start
  const Node* current = currNode;
  ROS_INFO("RECREATE PATH:");
  // Begin reverse-constructing the path
  std::unordered_map<nodeIDType, const Node*>::iterator searchParents;
  while (!(current->id_ == start->id_)) {

    ROS_INFO("%s", current->id_.c_str());
    // Push the current robot pose onto the robot traj for RVIZ
    geometry_msgs::Pose robotPose;
    robotPose.position.x = (float)(current->robotX_)/multiplicativeFactor_;
    robotPose.position.y = (float)(current->robotY_)/multiplicativeFactor_;
    robotPose.position.z = 0.0;
    robotPose.orientation = tf::createQuaternionMsgFromYaw(degreeToRad(current->robotTheta_));
    robotTrajPoses.push_back(robotPose);

    // Push the current human pose onto the robot traj for RVIZ
    geometry_msgs::Pose humanPose;
    humanPose.position.x = (float)(current->humanX_)/multiplicativeFactor_;
    humanPose.position.y = (float)(current->humanY_)/multiplicativeFactor_;
    humanPose.position.z = 0.0;
    humanPose.orientation = tf::createQuaternionMsgFromYaw(degreeToRad(current->humanTheta_));
    humanTrajPoses.push_back(humanPose);

    // Push the current robot pose onto the robotPath to be returned
    robotPath.push_back(std::tuple<int, int, int>(current->robotX_, current->robotY_, current->robotTheta_));
    // ROS_WARN("robot=(%d, %d, %d), handle=(%d, %d, %d)", current->robotX_, current->robotY_, current->robotTheta_,current->handleX_, current->handleY_, current->handleTheta_);
    humanPath.push_back(std::tuple<int, int, int>(current->humanX_, current->humanY_, current->humanTheta_));

    int subsampleInterval = ceil(current->robotpath_.size()/float(numPointsToSubsample_+1));

    for (int i = current->robotpath_.size()-1; i >= 0; --i) {
      if (numPointsToSubsample_ > 0 && i % subsampleInterval == 0) {
        robotPath.push_back(std::tuple<int, int, int>(std::get<0>(current->robotpath_[i]), std::get<1>(current->robotpath_[i]), std::get<2>(current->robotpath_[i])));
      }
      exactRobotPath.push_back(current->robotpath_[i]);
      // ROS_INFO("Robot Exact %d | %d | %d", std::get<0>(current->robotpath_[i]), std::get<1>(current->robotpath_[i]), std::get<2>(current->robotpath_[i]));
    }

    for (int i = current->humanpath_.size()-1; i >= 0; --i) {
      exactHumanPath.push_back(current->humanpath_[i]);
      // ROS_INFO("Human Exact %d | %d | %d", std::get<0>(current->humanpath_[i]), std::get<1>(current->humanpath_[i]), std::get<2>(current->humanpath_[i]));
    }
    // exactHumanPath.push_back(std::tuple<int, int, int>(current->humanX_, current->humanY_, current->humanTheta_));
    // Find the current node's parent

    searchParents = parents_.find(current->id_);
    if (searchParents == parents_.end() || (searchParents->second) == nullptr) {
      ROS_ERROR("Node %s has no parent, this should not happen...", current->id_.c_str());
      return false;
    }
    current = searchParents->second;
  }
  ROS_INFO("%s", current->id_.c_str());

  // Add the start robot pose to be published to RVIZ
  geometry_msgs::Pose robotPose;
  robotPose.position.x = (float)(current->robotX_)/multiplicativeFactor_;
  robotPose.position.y = (float)(current->robotY_)/multiplicativeFactor_;
  robotPose.position.z = 0.0;
  robotPose.orientation = tf::createQuaternionMsgFromYaw(degreeToRad(current->robotTheta_));
  robotTrajPoses.push_back(robotPose);
  // Create the PoseArray to publish to RVIZ
  geometry_msgs::PoseArray robotTrajPoseArray;
  robotTrajPoseArray.header.stamp = ros::Time::now();
  robotTrajPoseArray.header.frame_id = "/map";
  robotTrajPoseArray.poses = robotTrajPoses;
  robot_traj_pub_.publish(robotTrajPoseArray);
  // Add the start human pose to be published to RVIZ
  geometry_msgs::Pose humanPose;
  humanPose.position.x = (float)(current->humanX_)/multiplicativeFactor_;
  humanPose.position.y = (float)(current->humanY_)/multiplicativeFactor_;
  humanPose.position.z = 0.0;
  humanPose.orientation = tf::createQuaternionMsgFromYaw(degreeToRad(current->humanTheta_));
  humanTrajPoses.push_back(humanPose);
  // Create the PoseArray to publish to RVIZ
  geometry_msgs::PoseArray humanTrajPoseArray;
  humanTrajPoseArray.header.stamp = ros::Time::now();
  humanTrajPoseArray.header.frame_id = "/map";
  humanTrajPoseArray.poses = humanTrajPoses;
  human_traj_pub_.publish(humanTrajPoseArray);
  // Add the start robot pose to the path to be returned
  robotPath.push_back(std::tuple<int, int, int>(current->robotX_, current->robotY_, current->robotTheta_));
  humanPath.push_back(std::tuple<int, int, int>(current->humanX_, current->humanY_, current->humanTheta_));

  // Also, publish the path as a nav_msgs Path (the PoseArray above will show
  // orientation, the nav_msgs::Path below will show a continuous path)

  nav_msgs::Path gui_path;
  gui_path.poses.resize(robotPath.size());
  gui_path.header.frame_id = "/map";
  gui_path.header.stamp = ros::Time::now();
  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for (unsigned int i = 0; i < robotPath.size(); i++) {
      gui_path.poses[i].header.stamp = ros::Time::now();
      gui_path.poses[i].header.frame_id = "/map";
      gui_path.poses[i].pose.position.x = (float)std::get<0>(robotPath[i])/multiplicativeFactor_;
      gui_path.poses[i].pose.position.y = (float)std::get<1>(robotPath[i])/multiplicativeFactor_;
      gui_path.poses[i].pose.position.z = 0;
      gui_path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(degreeToRad(std::get<2>(robotPath[i])));
  }
  robot_plan_pub_.publish(gui_path);


  // Also, publish the "exact path" taken with the curves the robot takes, just
  // FYI.
  gui_path.poses.resize(exactRobotPath.size());
  gui_path.header.frame_id = "/map";
  gui_path.header.stamp = ros::Time::now();
  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for (unsigned int i = 0; i < exactRobotPath.size(); ++i) {
      gui_path.poses[i].header.stamp = ros::Time::now();
      gui_path.poses[i].header.frame_id = "/map";
      gui_path.poses[i].pose.position.x = (float)std::get<0>(exactRobotPath[i])/multiplicativeFactor_;
      gui_path.poses[i].pose.position.y = (float)std::get<1>(exactRobotPath[i])/multiplicativeFactor_;
      gui_path.poses[i].pose.position.z = 0;
      gui_path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(degreeToRad(std::get<2>(exactRobotPath[i])));
  }
  exact_robot_plan_pub_.publish(gui_path);


  // Also, publish the path as a nav_msgs Path (the PoseArray above will show
  // orientation, the nav_msgs::Path below will show a continuous path)
  gui_path.poses.resize(humanPath.size());
  gui_path.header.frame_id = "/map";
  gui_path.header.stamp = ros::Time::now();
  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for (unsigned int i = 0; i < humanPath.size(); i++) {
      gui_path.poses[i].header.stamp = ros::Time::now();
      gui_path.poses[i].header.frame_id = "/map";
      gui_path.poses[i].pose.position.x = (float)std::get<0>(humanPath[i])/multiplicativeFactor_;
      gui_path.poses[i].pose.position.y = (float)std::get<1>(humanPath[i])/multiplicativeFactor_;
      gui_path.poses[i].pose.position.z = 0;
      gui_path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(degreeToRad(std::get<2>(humanPath[i])));
  }
  human_plan_pub_.publish(gui_path);


  // Also, publish the "exact path" taken with the curves the robot takes, just
  // FYI.
  gui_path.poses.resize(exactHumanPath.size());
  gui_path.header.frame_id = "/map";
  gui_path.header.stamp = ros::Time::now();
  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for (unsigned int i = 0; i < exactHumanPath.size(); ++i) {
      gui_path.poses[i].header.stamp = ros::Time::now();
      gui_path.poses[i].header.frame_id = "/map";
      gui_path.poses[i].pose.position.x = (float)std::get<0>(exactHumanPath[i])/multiplicativeFactor_;
      gui_path.poses[i].pose.position.y = (float)std::get<1>(exactHumanPath[i])/multiplicativeFactor_;
      gui_path.poses[i].pose.position.z = 0;
      gui_path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(degreeToRad(std::get<2>(exactHumanPath[i])));
  }

  exact_human_plan_pub_.publish(gui_path);


  return true;
}

bool Planner::recreatePathSearch(const Node* start, const Node* currNode) {
  // ROS_ERROR("About to publish search path 0");
  // start from the currNode (which is close to the goal) and work backwards
  // to the start
  const Node* current = currNode;
  // Create the msg
  nav_msgs::Path gui_path;
  gui_path.poses.clear();
  gui_path.header.frame_id = "/map";
  gui_path.header.stamp = ros::Time::now();
  // Begin reverse-constructing the path

  std::unordered_map<nodeIDType, const Node*>::iterator searchParents;
  while (!(current->id_ == start->id_)) {
    // Push the current robot pose onto the robot traj for RVIZ
    geometry_msgs::PoseStamped robotPose;
    robotPose.header.stamp = ros::Time::now();
    robotPose.header.frame_id = "/map";
    robotPose.pose.position.x = (float)(current->robotX_)/multiplicativeFactor_;
    robotPose.pose.position.y = (float)(current->robotY_)/multiplicativeFactor_;
    robotPose.pose.position.z = 0;
    robotPose.pose.orientation = tf::createQuaternionMsgFromYaw(degreeToRad(current->robotTheta_));
    // Push the current robot pose onto the robotPath to be returned
    gui_path.poses.push_back(robotPose);
    // Find the current node's parent
    searchParents = parents_.find(current->id_);

    if (searchParents == parents_.end() || (searchParents->second) == nullptr) {
      ROS_ERROR("Node %s has no parent, this should not happen...", current->id_.c_str());
      return false;
    }
    current = searchParents->second;
  }
  // Add the start robot pose to be published to RVIZ
  geometry_msgs::PoseStamped robotPose;
  robotPose.header.stamp = ros::Time::now();
  robotPose.header.frame_id = "/map";
  robotPose.pose.position.x = (float)(current->robotX_)/multiplicativeFactor_;
  robotPose.pose.position.y = (float)(current->robotY_)/multiplicativeFactor_;
  robotPose.pose.position.z = 0;
  robotPose.pose.orientation = tf::createQuaternionMsgFromYaw(degreeToRad(current->robotTheta_));
  gui_path.poses.push_back(robotPose);

  // ROS_ERROR("About to publish search path 1");
  search_robot_plan_pub_.publish(gui_path);

  return true;
}

} //end namespace podi_robot_human_coupled_planner
