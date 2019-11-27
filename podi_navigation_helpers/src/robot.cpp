#include <tuple>
#include <vector>
#include <functional>
#include <math.h>

#include <ros/console.h>

#include <podi_navigation_helpers/robot.h>

namespace podi_navigation_helpers {

Robot::Robot(float handleLength) {
  handleLength_ = handleLength;
}

Robot::Robot(float handleLength, std::vector<geometry_msgs::Point> footprint,
  int maxV, int minV, int maxW, int minW, int minInPlaceW, int maxLinAcc, int maxRotAcc,
  double simTime, int linSimGranularity, int rotSimGranularity){
  handleLength_ = handleLength;
  footprint_ = footprint;
  maxV_ = maxV;
  minV_ = minV;
  maxW_ = maxW;
  minW_ = minW;
  minInPlaceW_ = minInPlaceW;
  maxLinAcc_ = maxLinAcc;
  maxRotAcc_ = maxRotAcc;
  simTime_ = simTime;
  linSimGranularity_ = linSimGranularity;
  rotSimGranularity_ = rotSimGranularity;
}

// Round normally -- note that integer division in C++ rounds towards 0
int Robot::round(int n, int roundTo) const {
  if (roundTo == 0) {
    ROS_ERROR("Robot round expects a positive number for roundTo, got 0.");
    ROS_ERROR("Rounding will not work");
    return n;
  }
  if (roundTo < 0) {
    ROS_ERROR("Robot round expects a positive number for roundTo, got %d.", roundTo);
    ROS_ERROR("Taking the abs of roundTo");
    roundTo = abs(roundTo);
  }
  if (n >= 0) {
    if (2*(n % roundTo) >= roundTo) {
      return ((n/roundTo)+1)*roundTo;
    }
    return (n/roundTo)*roundTo;
  } else {
    if (-2*(n % roundTo) >= roundTo) {
      return ((n/roundTo)-1)*roundTo;
    }
    return (n/roundTo)*roundTo;
  }
}

// Comes from trajectory_planner.cpp in the local planner
std::vector<std::tuple<int, int, int, int, int, std::vector<std::tuple<int, int, int, int, int> > > > Robot::getValidMoves(
  int x, int y, int theta, int v, int w, int vDiscretization, int wDiscretization, int vSamples, int wSamples) const {
  std::vector<std::tuple<int, int, int, int, int, std::vector<std::tuple<int, int, int, int, int> > > > retVal;
  // Calculate the parameters of the search
  // Don't allow for backwards motions
  int maxV = std::max(std::min(maxV_, (int)(v + maxLinAcc_ * simTime_)), minV_);
  int minV = std::min(std::max(minV_, (int)(v - maxLinAcc_ * simTime_)), maxV_);
  int maxW = std::max(std::min(maxW_, (int)(w + maxRotAcc_ * simTime_)), minW_);
  int minW = std::min(std::max(minW_, (int)(w - maxRotAcc_ * simTime_)), maxW_);
  int dv = (maxV - minV)/(vSamples - 1);
  int dw = (maxW - minW)/(wSamples - 1);

  // First sample the 0 velocities
  int vSamp = 0;
  int wSamp = 0;

  // Loop over all velocities
  for (int i = 0; i < vSamples; ++i) { // loops over v
    wSamp = 0.0;
    for (int j = 0; j < wSamples; ++j) { // loops over w
      int wSampModified = wSamp;
      if (i == 0 && j > 0) { // The base can't handle small in-place rotations
        wSampModified = wSamp >= 0 ? std::max(wSamp, minInPlaceW_) : std::min(wSamp, -1 * minInPlaceW_);
      }

      // Determine the length of each interval we are simulating the robot motion for
      int numSteps = int(std::max(abs(vSamp * simTime_) / linSimGranularity_, abs(wSamp * simTime_) / rotSimGranularity_) + 0.5);
      if (numSteps == 0) numSteps == 1;
      double dt = simTime_ / numSteps;

      // Get the newX, newY, newTheta
      int newV = v;
      int newW = w;
      int newX = x;
      int newY = y;
      int newTheta = theta;
      std::vector<std::tuple<int, int, int, int, int> > path = {std::make_tuple(newX, newY, newTheta, newV, newW)};
      double timeRemaining = simTime_;
      for (int k = 0; k < numSteps - 1; ++k) { // loops over intervals at which we are calculating robot pose
        newV = computeNewVelocity(vSamp, newV, maxLinAcc_, dt, timeRemaining);
        newW = computeNewVelocity(wSamp, newW, maxRotAcc_, dt, timeRemaining);

        newX = computeNewXPosition(newX, newV, 0.0, degreeToRad(newTheta), dt);
        newY = computeNewYPosition(newY, newV, 0.0, degreeToRad(newTheta), dt);
        newTheta = computeNewThetaPosition(newTheta, newW, dt);

        path.push_back(std::make_tuple(newX, newY, newTheta, newV, newW));

        timeRemaining = timeRemaining - dt;
      }

      // Push this valid move onto the return value
      if(!(newV == 0 && newW ==0)) retVal.push_back(std::make_tuple(newX, newY, newTheta, newV, newW, path));

      // Increment vSamp and wSamp
      if (j == 0) { // straight trajectory
        wSamp = minW;
      } else {
        wSamp += dw;
      }
      // wSamp = std::round(wSamp/wDiscretization)*wDiscretization;
    }
    if (i == 0) { // in-place rotation
      vSamp = minV;
    } else {
      vSamp += dv;
    }
    // vSamp = std::round(vSamp/vDiscretization)*vDiscretization;
  }

  return retVal;
}

// Get the handle pose, given a robot pose
std::tuple<int, int, int> Robot::getHandlePosition(int x, int y, int theta) const {
  return std::make_tuple(x-(int)handleLength_*cos(degreeToRad(theta)), y-(int)handleLength_*sin(degreeToRad(theta)), theta);
}

// Get the handle pose, given a robot pose
std::tuple<float, float, float> Robot::getHandlePosition(float x, float y, float theta) const {
  return std::make_tuple(x-handleLength_*cos(degreeToRad(theta)), y-handleLength_*sin(degreeToRad(theta)), theta);
}

} //end namespace podi_navigation_helpers
