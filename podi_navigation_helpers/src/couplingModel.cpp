#include <cmath>

#include <ros/console.h>

#include <podi_navigation_helpers/couplingModel.h>
#include <geometry_msgs/Point32.h>

namespace podi_navigation_helpers {

CouplingModel::CouplingModel(double handleLength, double regionOffset, double regionSize,
  double regionAngle, double alpha, double beta, double distanceEqualityThreshold, double orientationEqualityThreshold, int intervalsWhenDrawingCurves, int multiplicativeFactor) {

  handleLength_ = handleLength;
  regionOffset_ = regionOffset;
  regionSize_ = regionSize;
  regionAngle_ = regionAngle;
  alpha_ = alpha;
  beta_ = beta;
  intervalsWhenDrawingCurves_ = intervalsWhenDrawingCurves;
  multiplicativeFactor_ = multiplicativeFactor;

  distanceEqualityThreshold_ = distanceEqualityThreshold;
  orientationEqualityThreshold_ = orientationEqualityThreshold;

  ROS_WARN("Starting coupling model with params: handleLength=%f, regionOffset=%f, regionSize=%f, regionAngle=%f, alpha=%f, beta=%f, intervalsWhenDrawingCurves=%d, multiplicativeFactor=%d", handleLength, regionOffset, regionSize,
   regionAngle, alpha, beta, intervalsWhenDrawingCurves, multiplicativeFactor);

}

CouplingModel::~CouplingModel() {

}

std::vector<std::tuple<double, double, double> > CouplingModel::getHumanTraj(double humanX, double humanY, double humanTheta, std::vector<std::tuple<double, double, double> > robotTraj) const {

  std::vector<std::tuple<double, double, double> > retVal;
  std::tuple<double, double, double> humanPose;
  // humanX = roundToFactor(humanX, distanceEqualityThreshold_);
  // humanY = roundToFactor(humanY, distanceEqualityThreshold_);
  // humanTheta = roundToFactor(humanTheta, orientationEqualityThreshold_);
  retVal.push_back(std::make_tuple(humanX, humanY, humanTheta));

  // Iterate over the whole robot trajectory, getting the human pose at
  // every step
  double oldRobotX, oldRobotY, oldRobotTheta, oldHumanX, oldHumanY, oldHumanTheta;
  double newRobotX, newRobotY, newRobotTheta, newHumanX, newHumanY, newHumanTheta;

  if (robotTraj.size() > 0) {
    oldRobotX = std::get<0>(robotTraj[0]);
    oldRobotY = std::get<1>(robotTraj[0]);
    oldRobotTheta = std::get<2>(robotTraj[0]);
    oldHumanX = humanX;
    oldHumanY = humanY;
    oldHumanTheta = humanTheta;
  }
  for (int i = 0; i < robotTraj.size(); ++i) {
    newRobotX = std::get<0>(robotTraj[i]);
    newRobotY = std::get<1>(robotTraj[i]);
    newRobotTheta = std::get<2>(robotTraj[i]);
    humanPose = getHumanPose(oldRobotX, oldRobotY, oldRobotTheta, oldHumanX, oldHumanY, oldHumanTheta, newRobotX, newRobotY, newRobotTheta);

    newHumanX = std::get<0>(humanPose);
    newHumanY = std::get<1>(humanPose);
    newHumanTheta = std::get<2>(humanPose);
    // newHumanX = roundToFactor(newHumanX, distanceEqualityThreshold_);
    // newHumanY = roundToFactor(newHumanY, distanceEqualityThreshold_);
    // newHumanTheta = roundToFactor(newHumanTheta, orientationEqualityThreshold_);
    retVal.push_back(std::make_tuple(newHumanX, newHumanY, newHumanTheta));
    oldRobotX = newRobotX;
    oldRobotY = newRobotY;
    oldRobotTheta = newRobotTheta;
    oldHumanX = newHumanX;
    oldHumanY = newHumanY;
    oldHumanTheta = newHumanTheta;
  }

  return retVal;
}

std::tuple<double, double, double> CouplingModel::getInitialHumanPose(
  double robotX, double robotY, double robotTheta) const {
  double x = robotX - (handleLength_ + regionOffset_ + regionSize_ / 2.0) * std::cos(degreeToRad(robotTheta));
  double y = robotY - (handleLength_ + regionOffset_ + regionSize_ / 2.0) * std::sin(degreeToRad(robotTheta));
  double theta = robotTheta;
  // x = roundToFactor(x, distanceEqualityThreshold_);
  // y = roundToFactor(y, distanceEqualityThreshold_);
  // theta = roundToFactor(theta, orientationEqualityThreshold_);
  return std::make_tuple(x, y, theta);
}

geometry_msgs::PolygonStamped CouplingModel::drawRegion(double robotX,
  double robotY, double robotTheta, std::string frame_id) const {

  geometry_msgs::PolygonStamped retVal;

  double cx = robotX - (handleLength_) * std::cos(degreeToRad(robotTheta));
  double cy = robotY - (handleLength_) * std::sin(degreeToRad(robotTheta));
  double minRadius = regionOffset_;
  double maxRadius = regionOffset_ + regionSize_;
  double startingAngle = robotTheta + 180.0 - regionAngle_ / 2.0;
  normalizeAngle(startingAngle);
  double regionAngle = regionAngle_;

  retVal.polygon.points.clear();
  int intervals = intervalsWhenDrawingCurves_;
  for (int i = 0; i <= intervals; ++i) {
    geometry_msgs::Point32 point;
    point.x = (cx + minRadius*std::cos(degreeToRad(startingAngle + regionAngle*i/intervals)))/multiplicativeFactor_;
    point.y = (cy + minRadius*std::sin(degreeToRad(startingAngle + regionAngle*i/intervals)))/multiplicativeFactor_;
    point.z = 0.0;
    retVal.polygon.points.push_back(point);
  }
  for (int i = 0; i <= intervals; ++i) {
    geometry_msgs::Point32 point;
    point.x = (cx + maxRadius*std::cos(degreeToRad(startingAngle + regionAngle*(intervals-i)/intervals)))/multiplicativeFactor_;
    point.y = (cy + maxRadius*std::sin(degreeToRad(startingAngle + regionAngle*(intervals-i)/intervals)))/multiplicativeFactor_;
    point.z = 0.0;
    retVal.polygon.points.push_back(point);
  }

  retVal.header.frame_id = frame_id;
  retVal.header.stamp = ros::Time::now();

  return retVal;
}

std::tuple<double, double, double> CouplingModel::getHumanPose(
  double oldRobotX, double oldRobotY, double oldRobotTheta, double oldHumanX,
  double oldHumanY, double oldHumanTheta, double newRobotX, double newRobotY,
  double newRobotTheta) const {

  std::tuple<double, double> newHumanPos = closestPointInRegion(newRobotX, newRobotY, newRobotTheta, oldHumanX, oldHumanY, oldHumanTheta);
  double pullPointX, pullPointY, pullPointTheta;
  pullPointX = std::get<0>(newHumanPos);
  pullPointY = std::get<1>(newHumanPos);
  // angle to handle from pullPoint
  double angleToHandle = radToDegree(std::atan2((newRobotY - handleLength_ * std::sin(degreeToRad(newRobotTheta))) - pullPointY, (newRobotX - handleLength_ * std::cos(degreeToRad(newRobotTheta))) - pullPointX));
  normalizeAngle(angleToHandle);
  // angle of motion to pullPoint
  double angleOfMotion;
  if (fEq(pullPointX, oldHumanX, distanceEqualityThreshold_) && fEq(pullPointY, oldHumanY, distanceEqualityThreshold_)) {
    angleOfMotion = oldHumanTheta;
  } else {
    angleOfMotion = radToDegree(std::atan2(pullPointY - oldHumanY, pullPointX - oldHumanX));
  }
  normalizeAngle(angleOfMotion);
  pullPointTheta = weightedAngleAvg(alpha_, angleToHandle, 1.0 - alpha_, angleOfMotion);

  double compensationPointX, compensationPointY, compensationPointTheta;
  // distance from old human pose to old handle pose
  double dist = std::pow(std::pow(oldHumanX - (oldRobotX - handleLength_ * std::cos(degreeToRad(oldRobotTheta))), 2.0) + std::pow(oldHumanY - (oldRobotY - handleLength_ * std::sin(degreeToRad(oldRobotTheta))), 2.0), 0.5);
  compensationPointX = (newRobotX - (dist + handleLength_) * std::cos(degreeToRad(newRobotTheta)));
  compensationPointY = (newRobotY - (dist + handleLength_) * std::sin(degreeToRad(newRobotTheta)));
  // angle to handle from compensationPoint
  compensationPointTheta = radToDegree(std::atan2((newRobotY - handleLength_ * std::sin(degreeToRad(newRobotTheta))) - compensationPointY, (newRobotX - handleLength_ * std::cos(degreeToRad(newRobotTheta))) - compensationPointX));

  // Set the predicted human angle to the weighted average of the jsonPathToWriteAvgPoses
  double humanThetaNew = weightedAngleAvg(beta_, pullPointTheta, 1.0 - beta_, compensationPointTheta);
  double humanXNew = beta_ * pullPointX + (1.0 - beta_) * compensationPointX;
  double humanYNew = beta_ * pullPointY + (1.0 - beta_) * compensationPointY;

  // humanXNew = roundToFactor(humanXNew, distanceEqualityThreshold_);
  // humanYNew = roundToFactor(humanYNew, distanceEqualityThreshold_);
  // humanThetaNew = roundToFactor(humanThetaNew, orientationEqualityThreshold_);

  return std::tuple<double, double, double>(humanXNew, humanYNew, humanThetaNew);
}

std::tuple<double, double> CouplingModel::closestPointInRegion(double robotX, double robotY, double robotTheta, double humanX, double humanY, double humanTheta) const {
  double handleX = robotX - handleLength_ * std::cos(degreeToRad(robotTheta));
  double handleY = robotY - handleLength_ * std::sin(degreeToRad(robotTheta));
  // Take care of the edge case where human and handle are at the same location
  if (fEq(humanX, handleX, distanceEqualityThreshold_) && fEq(humanY, handleY, distanceEqualityThreshold_)) {
    // ROS_WARN("Human and handle at same point");
    double x = handleX + regionOffset_ * std::cos(degreeToRad(robotTheta));
    double y = handleY + regionOffset_ * std::sin(degreeToRad(robotTheta));
    return std::make_tuple(x, y);
  }
  // calculate the angle and distanceSq of the line segment from p to the center of c
  double distanceSq = std::pow(humanX-handleX, 2.0)+std::pow(humanY-handleY, 2.0);
  double angle = radToDegree(std::atan2(humanY-handleY, humanX-handleX));
  normalizeAngle(angle);
  // calculate the ending angle and middle angle of the region
  double startingAngle = robotTheta + 180.0 - regionAngle_ / 2.0;
  normalizeAngle(startingAngle);
  double middleAngle = robotTheta; // Point 180 degrees rotated from the center of the region
  normalizeAngle(middleAngle);
  double endingAngle = robotTheta + 180.0 + regionAngle_ / 2.0;
  normalizeAngle(endingAngle);

  // if angle is between starting angle and endings angle
  if ((fGeq(endingAngle, startingAngle, orientationEqualityThreshold_) && fGeq(angle, startingAngle, orientationEqualityThreshold_) && fLeq(angle, endingAngle, orientationEqualityThreshold_)) ||
      (fLeq(endingAngle, startingAngle, orientationEqualityThreshold_) && (fGeq(angle, startingAngle, orientationEqualityThreshold_) || fLeq(angle, endingAngle, orientationEqualityThreshold_)))) {
    double minRadiusSq = std::pow(regionOffset_, 2.0);
    double maxRadiusSq = std::pow(regionSize_+regionOffset_, 2.0);
    // ROS_WARN("distanceSq=%f, minRadiusSq=%f, maxRadiusSq=%f", distanceSq, minRadiusSq, maxRadiusSq);
    if (fLeq(distanceSq, minRadiusSq, distanceEqualityThreshold_)) {
      // ROS_WARN("Closest point on small arc");
      // the closest point is the point on the smaller circular arc of c that is closest to p
      double x = handleX + regionOffset_ * std::cos(degreeToRad(angle));
      double y = handleY + regionOffset_ * std::sin(degreeToRad(angle));
      return std::tuple<double, double>(x, y);

    } else if (fLess(distanceSq, maxRadiusSq, distanceEqualityThreshold_)) {
      // ROS_WARN("Human is already in the region");
      // the human is already inside the region
      return std::tuple<double, double>(humanX, humanY);

    } else {
      // ROS_WARN("Closest point is on large arc");
      // the closest point is the point on the larger circular arc of c that is closest to p
      double x = handleX + (regionOffset_ + regionSize_) * std::cos(degreeToRad(angle));
      double y = handleY + (regionOffset_ + regionSize_) * std::sin(degreeToRad(angle));
      return std::tuple<double, double>(x, y);

    }
  } else {
    double p0x = humanX;
    double p0y = humanY;
    double p1x, p1y, p2x, p2y;
    // if angle is on the far side of startingAngle
    // ROS_ERROR("startingAngle=%f, middleAngle=%f, endingAngle=%f, angle=%f", startingAngle, middleAngle, endingAngle, angle);
    if ((fGeq(startingAngle, middleAngle, orientationEqualityThreshold_) && fGeq(angle, middleAngle, orientationEqualityThreshold_) && fLeq(angle, startingAngle, orientationEqualityThreshold_)) ||
         (fLeq(startingAngle, middleAngle, orientationEqualityThreshold_) && (fGeq(angle, middleAngle, orientationEqualityThreshold_) || fLeq(angle, startingAngle, orientationEqualityThreshold_)))) {
      // ROS_WARN("Closest point is on starting line");
      // Closest Point is on the Starting Line
      p1x = handleX + regionOffset_ * std::cos(degreeToRad(startingAngle));
      p1y = handleY + regionOffset_ * std::sin(degreeToRad(startingAngle));
      p2x = handleX + (regionOffset_ + regionSize_) * std::cos(degreeToRad(startingAngle));
      p2y = handleY + (regionOffset_ + regionSize_) * std::sin(degreeToRad(startingAngle));
    // if angle is on the far side of endingAngle
    } else {
      // ROS_WARN("Closest point is on ending line");
      // Closest Point is on the Ending Line
      p1x = handleX + regionOffset_ * std::cos(degreeToRad(endingAngle));
      p1y = handleY + regionOffset_ * std::sin(degreeToRad(endingAngle));
      p2x = handleX + (regionOffset_ + regionSize_) * std::cos(degreeToRad(endingAngle));
      p2y = handleY + (regionOffset_ + regionSize_) * std::sin(degreeToRad(endingAngle));
    }
    double t = (((p0x-p1x)*(p2x-p1x)+(p0y-p1y)*(p2y-p1y)))/(std::pow(p2x-p1x, 2.0)+std::pow(p2y-p1y, 2.0));
    // ROS_WARN("t=%f", t);
    t = std::min((double)1.0,std::max((double)0.0, t));
    double minPointX = p1x*(1.0-t)+p2x*t;
    double minPointY = p1y*(1.0-t)+p2y*t;
    return std::tuple<double, double>(minPointX, minPointY);
  }

//   // If the angle is between start and end Angle
//   if ((fGeq(endingAngle, startingAngle, orientationEqualityThreshold_) && fGeq(angle, startingAngle, orientationEqualityThreshold_) && fLeq(angle, endingAngle, orientationEqualityThreshold_)) ||
//       (fLeq(endingAngle, startingAngle, orientationEqualityThreshold_) && (fGeq(angle, startingAngle, orientationEqualityThreshold_) || fLeq(angle, endingAngle, orientationEqualityThreshold_)))) {
//       // Next point is distOfNewPoint away, same angle
//       double x = handleX + distOfNewPoint * std::cos(degreeToRad(angle));
//       double y = handleY + distOfNewPoint * std::sin(degreeToRad(angle));
//       return std::tuple<double, double>(x, y);
//   // If the angle is on the far side of the start angle
// } else if ((fGeq(startingAngle, middleAngle, orientationEqualityThreshold_) && fGeq(angle, middleAngle, orientationEqualityThreshold_) && fLeq(angle, startingAngle, orientationEqualityThreshold_)) ||
//          (fLeq(startingAngle, middleAngle, orientationEqualityThreshold_) && (fGeq(angle, middleAngle, orientationEqualityThreshold_) || fLeq(angle, startingAngle, orientationEqualityThreshold_)))) {
//          // Next point is distOfNewPoint away, along startingAngle
//          double x = handleX + distOfNewPoint * std::cos(degreeToRad(startingAngle));
//          double y = handleY + distOfNewPoint * std::sin(degreeToRad(startingAngle));
//          return std::tuple<double, double>(x, y);
//   // If the angle is on the far side of the end angle
//   } else {
//     // Next point is distOfNewPoint away, along endingAngle
//     double x = handleX + distOfNewPoint * std::cos(degreeToRad(endingAngle));
//     double y = handleY + distOfNewPoint * std::sin(degreeToRad(endingAngle));
//     return std::tuple<double, double>(x, y);
//   }
}

} //end namespace podi_interaction_aware_planner
