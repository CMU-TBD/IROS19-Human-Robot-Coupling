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
#ifndef _INTERACTION_AWARE_PLANNER_H
#define _INTERACTION_AWARE_PLANNER_H

#include <algorithm> // for heap
#include <unordered_map>
#include <unordered_set>

#include <podi_robot_human_coupled_planner/node.h>

namespace podi_robot_human_coupled_planner {
class MinHeapElem {
    public:
        MinHeapElem(const Node* node, double cost) : node(node), cost(cost) {
        }
        const Node* node;
        double cost;
};

struct minHeapCmp {
        bool operator()(const MinHeapElem& a, const MinHeapElem& b) const {
            return a.cost > b.cost;
        }
};

class Planner {
    public:
        Planner(int multiplicativeFactor, int numPointsToSubsample, int heuristicRange,
          int heuristicRangeMultiplier, int heuristicRangeRequiredForSuccess, int intervalToPubPath, std::string name);
        ~Planner();
        bool getPath(const Node* start, std::vector<const Node*> goalNodes, int cycles,
          std::vector<std::tuple<int, int, int> >& robotPath, const Node*& endGoal);
        void clearDataStructures(std::vector<const Node*> nodesToKeep);
        void clearDataStructures() {
          std::vector<const Node*> nodesToKeep;
          clearDataStructures(nodesToKeep);
        };
    private:
        bool recreatePath(const Node* start, const Node* currNode,
          std::vector<std::tuple<int, int, int> >& robotPath);
          bool recreatePathSearch(const Node* start, const Node* currNode);
        std::vector<MinHeapElem> queue_;
        float degreeToRad(int degree) const;
        int multiplicativeFactor_, numPointsToSubsample_, heuristicRange_, heuristicRangeMultiplier_, heuristicRangeRequiredForSuccess_, intervalToPubPath_;
        std::unordered_map<nodeIDType, const Node*> parents_;
        std::unordered_map<nodeIDType, double> costs_;
        std::unordered_map<nodeIDType, const Node*> evaluated_;
        ros::Publisher human_traj_pub_, robot_traj_pub_, robot_plan_pub_, human_plan_pub_, search_robot_plan_pub_, exact_robot_plan_pub_,exact_human_plan_pub_;
};

} //end namespace podi_robot_human_coupled_planner
#endif
