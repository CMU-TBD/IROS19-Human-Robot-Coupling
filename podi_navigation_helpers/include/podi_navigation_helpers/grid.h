#ifndef _GRID_H
#define _GRID_H

#include <tuple>
#include <vector>

// A general class for an n-dimensional dynamic gridding of a space. Dynamic
// gridding means that the granularity of the grid is inversely
// proportional to the distance from that point to the goal(s).
// To allow for n-dimensionality, each point is an n-dimensional vector. The
// Grid assumes that the first two dimensions are x and y, and it calculates
// distance based on those two dimensions (using Euclidean distance).
// The other n-2 dimensions allow users to dynamically grid other aspects of
// the robot state (i.e. perhaps closer to the goal, the robot searches
// velocities with greater granularity).
namespace podi_navigation_helpers {

  class Grid {
      public:
        Grid(std::vector<std::vector<unsigned int> > gridSizes, std::vector<unsigned int> distanceThresholds,
          std::vector<int> gridOrigin);
        std::vector<std::vector<int> > setGoals(std::vector<std::vector<int> > goals);
        std::tuple<std::vector<int>, std::vector<unsigned int> > getGridCell(std::vector<int> pos) const;
      private:
        unsigned int distanceToGoal_(std::vector<int> cell) const;
        void checkGridSizesAndDistanceThresholds();
        unsigned int distance(int x, int y) const;
        int representativeValue(int val, int roundTo) const;
        std::vector<int> getGridCellHelper(std::vector<int> pos, std::vector<unsigned int> cellSize) const;
        std::vector<std::vector<int> > goalsRepresentativeElements_;
        std::vector<std::vector<unsigned int> > gridSizes_;
        std::vector<unsigned int> distanceThresholds_;
        std::vector<int> gridOrigin_;
        unsigned int n_;
  };

} //end namespace podi_navigation_helpers
#endif
