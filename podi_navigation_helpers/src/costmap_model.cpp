/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/
#include <podi_navigation_helpers/line_iterator.h>
#include <podi_navigation_helpers/costmap_model.h>
#include <costmap_2d/cost_values.h>

using namespace std;
using namespace costmap_2d;

/* NOTE (amal): I modified this file to encode the type of obstacles hit into
the return value. See the .h file for the actual values of LETHAL, INSCRIBED,
NOINFO, anmd OFFMAP */

namespace podi_navigation_helpers {
  CostmapModel::CostmapModel(const Costmap2D& ma) : costmap_(ma) {}

  // A negative return value indicates a problem, with the value indicating what
  // the problem is (more negative is more severe). A positive value indicates
  // that the pose is okay, with greater values meaning we are closer to an
  // obstacle.
  double CostmapModel::footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint,
      double inscribed_radius, double circumscribed_radius){
    double footprint_cost = 0.0;

    //used to put things into grid coordinates
    unsigned int cell_x, cell_y;

    //get the cell coord of the center point of the robot
    if(!costmap_.worldToMap(position.x, position.y, cell_x, cell_y)) {
      footprint_cost = OFFMAP;
    }
    //if number of points in the footprint is less than 3, we'll just assume a circular robot
    unsigned char cost = costmap_.getCost(cell_x, cell_y);
    if(cost == LETHAL_OBSTACLE) {
      footprint_cost = std::min(footprint_cost, LETHAL);
    }
    if(cost == NO_INFORMATION) {
      footprint_cost = std::min(footprint_cost, NOINFO);
    }
    if(cost == INSCRIBED_INFLATED_OBSTACLE) {
      footprint_cost = std::min(footprint_cost, INSCRIBED);
    }
    if(footprint.size() < 3){
      footprint_cost = footprint_cost < 0 ? footprint_cost : cost;
    }
    //now we really have to lay down the footprint in the costmap grid
    unsigned int x0, x1, y0, y1;
    double line_cost = 0.0;

    //we need to rasterize each line in the footprint
    for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
      //get the cell coord of the first point
      if(!costmap_.worldToMap(footprint[i].x, footprint[i].y, x0, y0)) {
        footprint_cost = std::min(footprint_cost, OFFMAP);
        continue;
      }
      //get the cell coord of the second point
      if(!costmap_.worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1)) {
        footprint_cost = std::min(footprint_cost, OFFMAP);
        continue;
      }
      line_cost = lineCost(x0, x1, y0, y1);
      // Negative values indicates a problem, and the more negative value, the
      // more severe the problem. Positive values means it is okay, but the more
      // positive the closer to an obstacle it is.
      if (line_cost < 0 || footprint_cost < 0) {
        footprint_cost = std::min(footprint_cost, line_cost);
      } else {
        footprint_cost = std::max(footprint_cost, line_cost);
      }
    }
    //we also need to connect the first point in the footprint to the last point
    //get the cell coord of the last point
    if(!costmap_.worldToMap(footprint.back().x, footprint.back().y, x0, y0)) {
      footprint_cost = std::min(footprint_cost, OFFMAP);
      return footprint_cost;
    }
    //get the cell coord of the first point
    if(!costmap_.worldToMap(footprint.front().x, footprint.front().y, x1, y1)) {
      footprint_cost = std::min(footprint_cost, OFFMAP);
      return footprint_cost;
    }
    line_cost = lineCost(x0, x1, y0, y1);
    // Negative values indicates a problem, and the more negative value, the
    // more severe the problem. Positive values means it is okay, but the more
    // positive the closer to an obstacle it is.
    if (line_cost < 0 || footprint_cost < 0) {
      footprint_cost = std::min(footprint_cost, line_cost);
    } else {
      footprint_cost = std::max(footprint_cost, line_cost);
    }
    // Return the overall cost
    return footprint_cost;

  }

  //calculate the cost of a ray-traced line
  double CostmapModel::lineCost(int x0, int x1,
      int y0, int y1){

    double line_cost = 0.0;
    double point_cost;
    int i = -1;
    for (podi_navigation_helpers::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance())
    {
      i++;
      point_cost = pointCost( line.getX(), line.getY() ); //Score the current point
      // Negative values indicates a problem, and the more negative value, the
      // more severe the problem. Positive values means it is okay, but the more
      // positive the closer to an obstacle it is.
      if (point_cost < 0 || line_cost < 0) {
        line_cost = std::min(line_cost, point_cost);
      } else {
        line_cost = std::max(line_cost, point_cost);
      }
    }

    return line_cost;
  }

  // NOTE (amal): I do not check whether the point is OFFMAP because I already
  // did for the two endpoints, and as long as the map is convex then if the
  // two endpoints are in the map, so are all points in between those endpoints.
  double CostmapModel::pointCost(int x, int y){
    unsigned char cost = costmap_.getCost(x, y);
    //if the cell is in an obstacle the path is invalid
    if(cost == LETHAL_OBSTACLE) {
      return LETHAL;
    }
    if(cost == INSCRIBED_INFLATED_OBSTACLE) {
      return INSCRIBED;
    }
    if(cost == NO_INFORMATION) {
      return NOINFO;
    }
    return cost;
  }

};
