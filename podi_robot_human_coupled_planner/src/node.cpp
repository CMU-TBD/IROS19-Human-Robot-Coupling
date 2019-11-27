#include <cmath>

#include <ros/console.h>

#include <podi_navigation_helpers/costmap_model.h>

#include <podi_robot_human_coupled_planner/node.h>

namespace podi_robot_human_coupled_planner {

Node::Node(podi_navigation_helpers::WorldModel* worldModel_,
  const podi_navigation_helpers::Grid* grid,
  const podi_navigation_helpers::Robot* robot, const podi_navigation_helpers::CouplingModel* couplingModel,
  int robotX, int robotY, int robotTheta, int v, int w,
  int multiplicativeFactor, double movementCost, double rotationCost, double obsCost,
  std::vector<geometry_msgs::Point> humanFootprint) : worldModel_(worldModel_), grid_(grid), robot_(robot), couplingModel_(couplingModel), humanFootprint_(humanFootprint) {

    int humanX, humanY, humanTheta;
    std::vector<std::tuple<int, int, int> > robotpath, humanpath;
    (this->init)(robotX, robotY, robotTheta, v, w, true, humanX, humanY, humanTheta, multiplicativeFactor, movementCost, rotationCost, obsCost, robotpath, humanpath);
}


Node::Node(podi_navigation_helpers::WorldModel* worldModel_,
  const podi_navigation_helpers::Grid* grid,
  const podi_navigation_helpers::Robot* robot, const podi_navigation_helpers::CouplingModel* couplingModel,
  int robotX, int robotY, int robotTheta, int v, int w,
  int humanX, int humanY, int humanTheta,
  int multiplicativeFactor, double movementCost, double rotationCost, double obsCost,
  std::vector<geometry_msgs::Point> humanFootprint) : worldModel_(worldModel_), grid_(grid), robot_(robot), couplingModel_(couplingModel), humanFootprint_(humanFootprint) {

    std::vector<std::tuple<int, int, int> > robotpath, humanpath;
    (this->init)(robotX, robotY, robotTheta, v, w, false, humanX, humanY, humanTheta, multiplicativeFactor, movementCost, rotationCost, obsCost, robotpath, humanpath);
}

Node::Node(podi_navigation_helpers::WorldModel* worldModel_,
  const podi_navigation_helpers::Grid* grid,
  const podi_navigation_helpers::Robot* robot, const podi_navigation_helpers::CouplingModel* couplingModel,
  int robotX, int robotY, int robotTheta, int v, int w,
  int humanX, int humanY, int humanTheta,
  int multiplicativeFactor, double movementCost, double rotationCost, double obsCost,
  std::vector<geometry_msgs::Point> humanFootprint,
  std::vector<std::tuple<int, int, int> > robotpath, std::vector<std::tuple<int, int, int> > humanpath
) : worldModel_(worldModel_), grid_(grid), robot_(robot), couplingModel_(couplingModel), humanFootprint_(humanFootprint) {

    (this->init)(robotX, robotY, robotTheta, v, w, false, humanX, humanY, humanTheta, multiplicativeFactor, movementCost, rotationCost, obsCost, robotpath, humanpath);
}

// Used only for the goal nodes
Node::Node(const podi_navigation_helpers::Grid* grid, int goalX, int goalY, int goalTheta, int goalV, int goalW) : grid_(grid) {
    // These three values are never used in planning, they are memerly so planner_core can recreate what our intended goal
    // (as opposed to the discretized goal) was from the node.
    robotX_ = goalX;
    robotY_ = goalY;
    robotTheta_ = goalTheta;
    std::tuple<std::vector<int>, std::vector<unsigned int> > humanXYTheta = grid_->getGridCell(std::vector<int>{goalX, goalY, goalTheta, goalV, goalW});
    humanX_ = std::get<0>(humanXYTheta)[0];
    humanY_ = std::get<0>(humanXYTheta)[1];
    humanTheta_ = (std::get<0>(humanXYTheta)[2] + 360) % 360; // nodes will have strictly positive theta
    v_ = std::get<0>(humanXYTheta)[3];
    w_ = std::get<0>(humanXYTheta)[4];
    id_ = std::to_string(humanX_) + " | " + std::to_string(humanY_) + " | " + std::to_string(humanTheta_) + " | " + std::to_string(v_) + " | " + std::to_string(w_);
}


void Node::init(int robotX, int robotY, int robotTheta, int v, int w, bool calcHumanPose, int humanX,
  int humanY, int humanTheta, int multiplicativeFactor, double movementCost,
  double rotationCost, double obsCost, std::vector<std::tuple<int, int, int> > robotpath,
  std::vector<std::tuple<int, int, int> > humanpath) {
  //ROS_ERROR("ROBOTPATHSIZE: %lu", robotpath.size());
  robotpath_ = robotpath;
  humanpath_ = humanpath;
  std::vector<int> robotLocation = {robotX, robotY, robotTheta, v, w};
  if (calcHumanPose) ROS_INFO("Actual robot x=%d, y=%d, th=%d", robotX, robotY, robotTheta);
  std::tuple<std::vector<int>, std::vector<unsigned int> > robotXYTheta = grid_->getGridCell(robotLocation);
  robotX_ = std::get<0>(robotXYTheta)[0];
  robotY_ = std::get<0>(robotXYTheta)[1];
  robotTheta_ = (std::get<0>(robotXYTheta)[2] + 360) % 360; // nodes will have strictly positive theta
  if (calcHumanPose) ROS_INFO("Discretized robot x=%d, y=%d, th=%d", robotX_, robotY_, robotTheta_);
  v_ = std::get<0>(robotXYTheta)[3];
  w_ = std::get<0>(robotXYTheta)[4];

  if (std::get<1>(robotXYTheta).size() == 0) {
    ROS_ERROR("Grid sizes is 0, possible unexpected behavior");
  } else {
    vDiscretization_ = std::get<1>(robotXYTheta)[3];
    wDiscretization_ = std::get<1>(robotXYTheta)[4];
    vSamples_ = std::get<1>(robotXYTheta)[5];
    wSamples_ = std::get<1>(robotXYTheta)[6];
  }


  // std::tuple<int, int, int> handleLocation = robot_->getHandlePosition(robotX_, robotY_, robotTheta_);
  // handleX_ = std::get<0>(handleLocation);
  // handleY_ = std::get<1>(handleLocation);
  // handleTheta_ = std::get<2>(handleLocation);

  if (calcHumanPose) {
    // ROS_ERROR("Handle  robot x=%d, y=%d, th=%d", handleX_, handleY_, handleTheta_);
    std::tuple<double, double, double> humanPose = couplingModel_->getInitialHumanPose((double)robotX_, (double)robotY_, (double)robotTheta_);
    humanX = std::get<0>(humanPose);
    humanY = std::get<1>(humanPose);
    humanTheta = std::get<2>(humanPose);
    ROS_INFO("Node init human x=%d, y=%d, theta=%d", humanX, humanY, humanTheta);
  }
  std::vector<int> humanLocation = {humanX, humanY, humanTheta};
  std::tuple<std::vector<int>, std::vector<unsigned int> > humanXYTheta = grid_->getGridCell(humanLocation);
  humanX_ = std::get<0>(humanXYTheta)[0];
  humanY_ = std::get<0>(humanXYTheta)[1];
  humanTheta_ = (std::get<0>(humanXYTheta)[2] + 360) % 360; // nodes will have strictly positive theta
  if (calcHumanPose) ROS_INFO("Discretized human x=%d, y=%d, th=%d", humanX_, humanY_, humanTheta_);
  // ROS_DEBUG("Node after creating discretized human x=%d, y=%d, th=%d", humanX_, humanY_, humanTheta_);

  multiplicativeFactor_ = multiplicativeFactor;

  id_ = std::to_string(robotX_) + " | " + std::to_string(robotY_) + " | " + std::to_string(robotTheta_) + " | " + std::to_string(humanX_) + " | " + std::to_string(humanY_) + " | " + std::to_string(humanTheta_) + " | " + std::to_string(v_) + " | " + std::to_string(w_);

  movementCost_ = movementCost;
  rotationCost_ = rotationCost;
  obsCost_ = obsCost;

}

std::vector<std::tuple<const Node*, double> > Node::neighbors() const {
  std::vector<std::tuple<const Node*, double> > retVal;

  // Loop over all valid robot moves
  std::vector<std::tuple<int, int, int, int, int, std::vector<std::tuple<int, int, int, int, int> > > > validMoves =
    robot_->getValidMoves(robotX_, robotY_, robotTheta_, v_, w_, vDiscretization_, wDiscretization_, vSamples_, wSamples_);
  double footprintCost;
  for (int i = 0; i < validMoves.size(); i++) {
    std::tuple<int, int, int, int, int, std::vector<std::tuple<int, int, int, int, int> > > move = validMoves[i];
    // Get the robot pose
    int robotx = std::get<0>(move);
    int roboty = std::get<1>(move);
    int robottheta = std::get<2>(move);
    // Get the robot velocities
    int v = std::get<3>(move);
    int w = std::get<4>(move);
    std::vector<std::tuple<int, int, int, int, int> > robotpath = std::get<5>(move);
    std::vector<std::tuple<int, int, int> > robotpathPoseOnly;
    std::vector<std::tuple<double, double, double> > robotpathPoseOnlyDouble;

    // Get the human pose
    std::tuple<int, int, int, int, int> robotpose;
    bool skipThisTrajectory = false;
    for (int k = 0; k < robotpath.size(); ++k) {
      robotpose = robotpath[k];
      // Ensure that no point collides with a trajectory
      footprintCost = getFootprintCost(std::get<0>(robotpose),std::get<1>(robotpose),std::get<2>(robotpose), robot_->footprint_);
      //ROS_ERROR("Robot Cost %i %i %i", std::get<0>(robotpose),std::get<1>(robotpose),std::get<2>(robotpose));
      //ROS_INFO("Node = %s, FootprintCost = %f", id_.c_str(), footprintCost);
      if (footprintCost < 0 && footprintCost != podi_navigation_helpers::INSCRIBED) { //&& footprintCost != podi_navigation_helpers::INSCRIBED
        skipThisTrajectory = true;
        break;
      }
      robotpathPoseOnly.push_back(std::make_tuple(std::get<0>(robotpose),std::get<1>(robotpose),std::get<2>(robotpose)));
      robotpathPoseOnlyDouble.push_back(std::make_tuple((double)std::get<0>(robotpose),(double)std::get<1>(robotpose),(double)std::get<2>(robotpose)));
    }
    if (skipThisTrajectory) continue;
    std::vector<std::tuple<double, double, double> > humanpathDouble = couplingModel_->getHumanTraj(humanX_, humanY_, humanTheta_, robotpathPoseOnlyDouble);
    std::vector<std::tuple<int, int, int> > humanpath;
    std::tuple<double, double, double> humanpose;
    skipThisTrajectory = false;
    for (int k = 0; k < humanpathDouble.size(); ++k) {
      // Ensure that no human point collides with an obstacle
      humanpose = humanpathDouble[k];
      footprintCost = getFootprintCost((int)std::round(std::get<0>(humanpose)), (int)std::round(std::get<1>(humanpose)), (int)std::round(std::get<2>(humanpose)), humanFootprint_);
      if (footprintCost < 0 && footprintCost != podi_navigation_helpers::INSCRIBED) { //&& footprintCost != podi_navigation_helpers::INSCRIBED
        //ROS_ERROR("Skipping this node");
        skipThisTrajectory = true;
        break;
      }
      //humanpose = humanpathDouble[k];
      humanpath.push_back(std::make_tuple((int)std::round(std::get<0>(humanpose)),
        (int)std::round(std::get<1>(humanpose)),(int)std::round(std::get<2>(humanpose))));
    }

    if (skipThisTrajectory) continue;

    int humanx = std::get<0>(humanpath[humanpath.size()-1]);
    int humany = std::get<1>(humanpath[humanpath.size()-1]);
    int humantheta = std::get<2>(humanpath[humanpath.size()-1]);
    // ROS_DEBUG("Node before creating human x=%d, y=%d, th=%d", humanx, humany, humantheta);
    // NOTE: we don't have to round to the grid here since Node does that in the
    // initializer
    // Generate the neighbor, and get its cost

    const Node* neighbor = const_cast<const Node*>(new Node(
      worldModel_, grid_, robot_, couplingModel_, robotx, roboty, robottheta, v, w, humanx, humany, humantheta, multiplicativeFactor_, movementCost_, rotationCost_, obsCost_, humanFootprint_, robotpathPoseOnly, humanpath));
    double cost = edgeCost(neighbor);

    retVal.push_back(std::make_tuple(neighbor, cost));

  }
  return retVal;
}

// The edge cost is a weighted sum of the distance traveled, the degrees rotated,
// and how close the point you are moving from is to an obstacle
double Node::edgeCost(const Node* neighbor) const {
  // Get the old position
  int oldX = humanX_;
  int oldY = humanY_;
  int oldTheta = humanTheta_;
  // Get the new position
  int newX = neighbor->humanX_;
  int newY = neighbor->humanY_;
  int newTheta = neighbor->humanTheta_;
  // Get the distance traveled, degrees rotated, and distance from obstacles (ish -- this is in exponentially decaying units)
  unsigned int totalDistance = distance(newX - oldX, newY - oldY);
  unsigned int totalRotation = abs(newTheta - oldTheta);
  totalRotation = std::min(totalRotation, 360-totalRotation);
  int robotFootprintCost = getFootprintCost(robotX_, robotY_, robotTheta_, robot_->footprint_);
  int humanFootprintCost = getFootprintCost(oldX, oldY, oldTheta, humanFootprint_);
  // Get the overall cost
  float cost = movementCost_*totalDistance + rotationCost_*totalRotation + obsCost_*(robotFootprintCost+humanFootprintCost)/2.0;

  return cost;
}

// Get the Euclidean distance from (0, 0) to (x, y), casting to an int
unsigned int Node::distance(int x, int y) const {
  return static_cast<unsigned int>(std::pow(std::pow(x, 2.0)+std::pow(y, 2.0), 0.5));
}

float Node::degreeToRad(int degree) const {
  return degree*M_PI/180;
}

bool Node::reachedGoal(const Node* goal) const {
  return (
    ((this->humanX_) == (goal->humanX_)) &&
    ((this->humanY_) == (goal->humanY_)) &&
    ((this->humanTheta_) == (goal->humanTheta_)) &&
    ((this->v_) == (goal->v_)) &&
    ((this->w_) == (goal->w_))
  );
}

// The edge cost is a weighted sum of the distance traveled, the degrees rotated,
// and how close the point you are moving from is to an obstacle. Note that this
// heuristic is consistent. I am incorproating obstacle cost because it makes
// the heuristic a little closer to the actual cost, even though since it is
// only based on one node, I don't think it really helps guide the search much.
double Node::heuristicToSingleGoal(const Node* goal) const {
  // Get the robot position
  int oldX = humanX_;
  int oldY = humanY_;
  int oldTheta = humanTheta_;
  // Get the goal position
  int newX = goal->humanX_;
  int newY = goal->humanY_;
  int newTheta = goal->humanTheta_;
  // Get the distance traveled, degrees rotated, and distance from obstacles (ish -- this is in exponentially decaying units)
  unsigned int totalDistance = distance(newX - oldX, newY - oldY);
  unsigned int totalRotation = abs(newTheta - oldTheta);
  totalRotation = std::min(totalRotation, 360-totalRotation);
  int robotFootprintCost = getFootprintCost(robotX_, robotY_, robotTheta_, robot_->footprint_);
  int humanFootprintCost = getFootprintCost(oldX, oldY, oldTheta, humanFootprint_);
  // Get the overall cost
  float cost = movementCost_*totalDistance + rotationCost_*totalRotation + obsCost_*(robotFootprintCost+humanFootprintCost)/2.0;
  return cost;
}

// Returns the min heuristic to the goals in the vector
double Node::heuristic(std::vector<const Node*> goalNodes) const {
  const Node* goal;
  return heuristic(goalNodes, goal);
}

// in addition to returning the heuristic, it sets goal to the goal we are
// are going towards (the closest goal)
double Node::heuristic(std::vector<const Node*> goalNodes, const Node*& goal) const {
  double minCost, cost;
  bool isMinCostSet = false;
  for (int i = 0; i < goalNodes.size(); ++i) {
    cost = heuristicToSingleGoal(goalNodes[i]);
    if (!isMinCostSet || cost < minCost) {
      minCost = cost;
      goal = goalNodes[i];
      isMinCostSet = true;
    }
  }
  return minCost;
}

} //end namespace podi_robot_human_coupled_planner
