#include <podi_navigation_helpers/grid.h>
#include <cmath>
#include <tuple>
#include <vector>
#include <ros/console.h>

namespace podi_navigation_helpers {

// Initialize the grid
Grid::Grid(std::vector<std::vector<unsigned int> > gridSizes, std::vector<unsigned int> distanceThresholds,
  std::vector<int> gridOrigin) {
  gridSizes_ = gridSizes;
  distanceThresholds_ = distanceThresholds;
  gridOrigin_ = gridOrigin;

  // Get the min dimensionality of elements of gridSize
  for (int i = 0; i < gridSizes_.size(); ++i) {
    if (i == 0) n_ = gridSizes_[i].size();
    if (gridSizes_[i].size() < n_) n_ = gridSizes_[i].size();
  }

  checkGridSizesAndDistanceThresholds();
}

// Perform multiple checks to ensure that gridSizes_ and distanceThresholds_
// are valid, and notify the user if not
void Grid::checkGridSizesAndDistanceThresholds() {
  // Must have at least one gridSize
  if (gridSizes_.size() < 1) {
    ROS_ERROR("Grid requires at least one grid size");
    ROS_ERROR("This grid will always return the same point it is given");
    return;
  }
  // gridSizes_ must have exactly one more element that distanceThresholds
  if (gridSizes_.size()-1 > distanceThresholds_.size()) {
    ROS_ERROR("Grid requires that gridSizes_.size()-1 == distanceThresholds_.size()");
    ROS_ERROR("Truncating gridSizes_");
    gridSizes_.resize(distanceThresholds_.size() + 1);
  }
  if (gridSizes_.size()-1 < distanceThresholds_.size()) {
    ROS_ERROR("Grid requires that gridSizes_.size()-1 == distanceThresholds_.size()");
    ROS_ERROR("Truncating distanceThresholds_");
    distanceThresholds_.resize(gridSizes_.size() - 1);
  }
  if (gridOrigin_.size() < n_) {
    ROS_ERROR("Grid requires that gridOrigin_.size()-1 == n_");
    ROS_ERROR("Filling gridOirigin_ in with 0s");
    for (int i = gridOrigin_.size(); i < n_; ++i)
      gridOrigin_.push_back(0);
  }
  if (gridOrigin_.size() > n_) {
    ROS_ERROR("Grid requires that gridOrigin_.size()-1 == n_");
    ROS_ERROR("Truncating gridOirigin_");
    gridOrigin_.resize(n_);
  }
  for (int i = 0; i < gridSizes_.size(); ++i) {
    // Every gridSize must have the same number of dimensions
    // Note that we define n as the min dimension of all gridSizes_
    if (gridSizes_[i].size() > n_) {
      ROS_ERROR("Grid requires that every gridSize has the same dimensionality");
      ROS_ERROR("Truncating gridSize[%d]", i);
      gridSizes_[i].resize(n_);
    }
    for (int k = 0; k < n_; ++k) {
      // If the first two elements of gridSize are <= 1, then we won't be able to properly subdivide
      if (k <= 1 && gridSizes_[i][k] <= 1) {
        ROS_ERROR("Grid requires that the first two elements of gridSize is > 1");
        ROS_ERROR("Grid will have undefined and potentially error-inducing behavior");
      }
      // Every element in a single gridSize must be positive
      if (gridSizes_[i][k] <= 0) {
        ROS_ERROR("Grid requires that every element of gridSize is positive");
        ROS_ERROR("Grid will have undefined and potentially error-inducing behavior");
      }
      // For only the first two elements of each gridSize:
      if (k < 2 && i > 0) {
        // Subsequent grid sizes of the same axis must not be larger than earlier ones
        if (gridSizes_[i-1][k] < gridSizes_[i][k]) {
          ROS_ERROR("Grid requires that for the first two dimensions, gridSizes_ along a single dimension be non-increasing");
          ROS_ERROR("Grid will have undefined and potentially error-inducing behavior");
        }
        // Subsequent grid sizes of the same axis must be factors of earlier ones
        if (gridSizes_[i-1][k] % gridSizes_[i][k] != 0) {
          ROS_ERROR("Grid requires that for the first two dimensions, gridSizes_ along a single dimension be factors of earlier ones");
          ROS_ERROR("Grid will have undefined and potentially error-inducing behavior");
        }
      }
    }
    if (i > 0) {
      // Every distanceThreshold must be > 0
      if (distanceThresholds_[i-1] <= 1) {
        ROS_ERROR("Grid requires that every distanceThresholds_ is > 1");
        ROS_ERROR("Grid will have undefined and potentially error-inducing behavior");
      }
      // distanceThresholds_ must be non-increasing
      if (i < distanceThresholds_.size() && distanceThresholds_[i-1] < distanceThresholds_[i]) {
        ROS_ERROR("Grid requires the distanceThresholds_ be non-increasing");
        ROS_ERROR("Grid will have undefined and potentially error-inducing behavior");
      }
      // The distanceThreshold at index i-1 must be > the diameter of the gridSize
      // at index i-1. This guarentees that if the goal is in a cell, then
      // regardless of what the representative element of the cell is,
      // we will always subdivide that cell to the next level if possible.
      unsigned int diameter = distance(gridSizes_[i-1][0], gridSizes_[i-1][1]);
      if (distanceThresholds_[i-1] <= diameter) {
        ROS_ERROR("Grid requires every distanceThreshold is > the diameter of the gridSize at the same index");
        ROS_ERROR("Grid will have undefined and potentially error-inducing behavior");
      }
    }
  }
}

// Get goals, convert them to representative elements in the finest gridSize,
// and store those as goals. Notw that although the goal can have more than two dimensions,
// we only ever use the first two dimensions
std::vector<std::vector<int> > Grid::setGoals(std::vector<std::vector<int> > goals) {
  if (gridSizes_.size() == 0) return goals;
  goalsRepresentativeElements_.clear();
  for (int i = 0; i < goals.size(); ++i) {
    goalsRepresentativeElements_.push_back(getGridCellHelper(goals[i], gridSizes_[gridSizes_.size()-1]));
  }
  return goalsRepresentativeElements_;
}

// Get the minDist between a cell and the representative elements for goal(s)
unsigned int Grid::distanceToGoal_(std::vector<int> cell) const {
  bool isMinDistSet = false;
  unsigned int minDist;
  for (int i = 0; i < goalsRepresentativeElements_.size(); ++i) {
    unsigned int dist = distance(goalsRepresentativeElements_[i][0]-cell[0],
                                 goalsRepresentativeElements_[i][1]-cell[1]);
    if (!isMinDistSet || dist < minDist) {
      minDist = dist;
      isMinDistSet = true;
    }
  }
  return minDist;
}

// Return approximately the middle element of the roundTo range that n is in.
// Inclusivity/exclusiuvity it determined towards 0. i.e. if n < 0, the range
// is inclusive of the upper bound, is n > 0 inclusive of lower bound,
// exclusive of the other. The reason it is approximately the middle element
// is due to int rounding errors.
// NOTE: For 0, we arbitrarily choose to round up instead of down.
int Grid::representativeValue(int val, int roundTo) const {
  // Round towards 0
  int towardsZeroVal = (val/roundTo)*roundTo;
  // Round away from 0
  int awayFromZeroVal = (val >= 0) ? towardsZeroVal + roundTo : towardsZeroVal - roundTo;
  // Average them to get the middle value
  return (towardsZeroVal+awayFromZeroVal)/2;
}

// Get the Euclidean distance from (0, 0) to (x, y), casting to an int
unsigned int Grid::distance(int x, int y) const {
  return static_cast<unsigned int>(std::pow(std::pow(x, 2.0)+std::pow(y, 2.0), 0.5));
}

// Returns the representative element of the cell of size cellSize that pos is
// in (gridded with respect to the origin). Note that we only get up to n
// dimensions, where n = pos.size(). Although ideally n = n_, users may desire
// to call getGridCellHelper with a pos of less dimensions
std::vector<int> Grid::getGridCellHelper(std::vector<int> pos, std::vector<unsigned int> cellSize) const {
  std::vector<int> retVal;
  unsigned int n = pos.size();
  for (unsigned int i = 0; i < n; i++) {
    retVal.push_back(representativeValue(pos[i]-gridOrigin_[i], cellSize[i])+gridOrigin_[i]);
  }
  return retVal;
}

// Gets the representative element of the gridCell that pos is in, along with
// the size of the grid it is in. Note that pos.size() can be < n_, but must
// be >= 2 (in order to calculate Euclidean distance).
std::tuple<std::vector<int>, std::vector<unsigned int> > Grid::getGridCell(std::vector<int> pos) const {
  if (gridSizes_.size() == 0) {
    std::vector<unsigned int> gridCell;
    return std::make_tuple(pos, gridCell);
  }
  unsigned int dist = 0;
  unsigned int currGridSizeIndex = 0;
  std::vector<int> cell;
  while (currGridSizeIndex == 0 || (currGridSizeIndex < gridSizes_.size() && dist < distanceThresholds_[currGridSizeIndex-1])) {
     cell = getGridCellHelper(pos, gridSizes_[currGridSizeIndex]);
     dist = distanceToGoal_(cell);
     currGridSizeIndex++;
  }
  return std::make_tuple(cell, gridSizes_[currGridSizeIndex-1]);
}

} //end namespace podi_navigation_helpers
