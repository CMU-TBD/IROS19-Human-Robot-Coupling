#ifndef _NODE_H
#define _NODE_H

#include <podi_navigation_helpers/robot.h>
#include <podi_navigation_helpers/couplingModel.h>
#include <podi_navigation_helpers/grid.h>
#include <podi_navigation_helpers/world_model.h>
#include <podi_navigation_helpers/robot.h>

namespace podi_robot_human_coupled_planner {

typedef std::string nodeIDType;

class Node {
  public:
    // Used only for start node
    Node(podi_navigation_helpers::WorldModel* worldModel,
      const podi_navigation_helpers::Grid* grid, const podi_navigation_helpers::Robot* robot,
      const podi_navigation_helpers::CouplingModel* couplingModel,
      int robotX, int robotY, int robotTheta, int v, int w,
      int multiplicativeFactor, double movementCost, double rotationCost, double obsCost,
      std::vector<geometry_msgs::Point> humanFootprint);
    Node(podi_navigation_helpers::WorldModel* worldModel,
      const podi_navigation_helpers::Grid* grid, const podi_navigation_helpers::Robot* robot,
      const podi_navigation_helpers::CouplingModel* couplingModel,
      int robotX, int robotY, int robotTheta, int v, int w, int humanX, int humanY, int humanTheta,
      int multiplicativeFactor, double movementCost, double rotationCost, double obsCost,
      std::vector<geometry_msgs::Point> humanFootprint);
    Node(podi_navigation_helpers::WorldModel* worldModel,
      const podi_navigation_helpers::Grid* grid, const podi_navigation_helpers::Robot* robot,
      const podi_navigation_helpers::CouplingModel* couplingModel,
      int robotX, int robotY, int robotTheta, int v, int w, int humanX, int humanY, int humanTheta,
      int multiplicativeFactor, double movementCost, double rotationCost, double obsCost,
      std::vector<geometry_msgs::Point> humanFootprint,
      std::vector<std::tuple<int, int, int> > robotpath, std::vector<std::tuple<int, int, int> > humanpath);
    // Used only for goal nodes
    Node(const podi_navigation_helpers::Grid* grid, int goalX, int goalY, int goalTheta, int goalV, int goalW);

    std::vector<std::tuple<const Node*, double> > neighbors() const;
    double heuristic(std::vector<const Node*> goalNodes) const;
    double heuristic(std::vector<const Node*> goalNodes, const Node*& goal) const;
    bool reachedGoal(const Node* goal) const;
    nodeIDType id_;
    const podi_navigation_helpers::Robot* robot_;
    int robotX_, robotY_, robotTheta_;
    // int handleX_, handleY_, handleTheta_;
    int humanX_, humanY_, humanTheta_;
    std::vector<std::tuple<int, int, int> > robotpath_;
    std::vector<std::tuple<int, int, int> > humanpath_;
  private:
    void init(int robotX, int robotY, int robotTheta, int v, int w, bool calcHumanPose,
      int humanX, int humanY, int humanTheta, int multiplicativeFactor,
      double movementCost, double rotationCost, double obsCost,
      std::vector<std::tuple<int, int, int> > robotpath, std::vector<std::tuple<int, int, int> > humanpath);
    double edgeCost(const Node* neighbor) const;
    double heuristicToSingleGoal(const Node* goal) const;
    double getFootprintCost(int x, int y, int theta, std::vector<geometry_msgs::Point> footprint) const {
      return worldModel_->footprintCost((double)x/(double)multiplicativeFactor_,
      (double)y/(double)multiplicativeFactor_, degreeToRad(theta), footprint);
    }
    float degreeToRad(int degree) const;
    unsigned int distance(int x, int y) const;
    podi_navigation_helpers::WorldModel* worldModel_;
    const podi_navigation_helpers::Grid* grid_;
    int v_, w_;
    int vSamples_, wSamples_;
    int vDiscretization_, wDiscretization_;
    int multiplicativeFactor_;
    float movementCost_, rotationCost_, obsCost_;
    const podi_navigation_helpers::CouplingModel* couplingModel_;
    std::vector<geometry_msgs::Point> humanFootprint_;
};

} //end namespace podi_robot_human_coupled_planner
#endif
