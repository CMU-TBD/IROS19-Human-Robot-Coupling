#ifndef _HUMAN_H
#define _HUMAN_H

#include <tuple>
#include <vector>
#include <cmath>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>

#include <limits>

namespace podi_navigation_helpers {

// NOTE: Distance units in the coupling model can be any units as long as it is
// cosnistent. Angle units must be degrees, since we convert to radians before
// taking cos/sin.
class CouplingModel {
public:
  CouplingModel() {};
  CouplingModel(double handleLength, double regionOffset, double regionSize,
    double regionAngle, double alpha, double beta, double distanceEqualityThreshold, double orientationEqualityThreshold, int intervalsWhenDrawingCurves, int multiplicativeFactor);
  CouplingModel(double handleLength, double regionOffset, double regionSize,
    double regionAngle, double alpha, double beta, double distanceEqualityThreshold, double orientationEqualityThreshold) {
      CouplingModel(handleLength, regionOffset, regionSize, regionAngle, alpha, beta, distanceEqualityThreshold, orientationEqualityThreshold, 1, 1);
    };
  ~CouplingModel();
  std::vector<std::tuple<double, double, double> > getHumanTraj(double humanX, double humanY, double humanTheta, std::vector<std::tuple<double, double, double> > robotTraj) const;
  std::tuple<double, double, double> getInitialHumanPose(double robotX, double robotY, double robotTheta) const;
  std::tuple<double, double, double> getHumanPose(double oldRobotX, double oldRobotY, double oldRobotTheta, double oldHumanX, double oldHumanY, double oldHumanTheta, double newRobotX, double newRobotY, double newRobotTheta) const;
  std::tuple<double, double> closestPointInRegion(double robotX, double robotY, double robotTheta, double humanX, double humanY, double humanTheta) const;
  geometry_msgs::PolygonStamped drawRegion(double robotX, double robotY, double robotTheta, std::string frame_id) const;
private:
  bool fEq(double x0, double x1, double epsilon) const {
    if (fabs(x0-x1) <= epsilon) return true;
    return false;
  };
  bool fGeq(double x0, double x1, double epsilon) const {
    if (x0 >= x1 - epsilon) return true;
    return false;
  };
  bool fGreat(double x0, double x1, double epsilon) const {
    if (x0 > x1 + epsilon) return true;
    return false;
  };
  bool fLeq(double x0, double x1, double epsilon) const {
    if (x0 <= x1 + epsilon) return true;
    return false;
  };
  bool fLess(double x0, double x1, double epsilon) const {
    if (x0 < x1 - epsilon) return true;
    return false;
  };
  bool fEq(double x0, double x1) const {
    double epsilon = std::numeric_limits<double>::epsilon();//0.00005;//
    if (fabs(x0-x1) <= epsilon) return true;
    return false;
  };
  bool fGeq(double x0, double x1) const {
    double epsilon = std::numeric_limits<double>::epsilon();//0.00005;//
    if (x0 >= x1 - epsilon) return true;
    return false;
  };
  bool fGreat(double x0, double x1) const {
    double epsilon = std::numeric_limits<double>::epsilon();//0.00005;//
    if (x0 > x1 + epsilon) return true;
    return false;
  };
  bool fLeq(double x0, double x1) const {
    double epsilon = std::numeric_limits<double>::epsilon();//0.00005;//
    if (x0 <= x1 + epsilon) return true;
    return false;
  };
  bool fLess(double x0, double x1) const {
    double epsilon = std::numeric_limits<double>::epsilon();//0.00005;//
    if (x0 < x1 - epsilon) return true;
    return false;
  };
  // double roundToFactor(double num, double factor) {
  //   return std::round(num/factor)*factor;
  // }
  double degreeToRad(double degree) const {
    return degree*M_PI/180.0;
  }
  double radToDegree(double rad) const {
    return rad*180.0/M_PI;
  }
  void normalizeAngle(double& theta) const {
    while (theta < 0.0) theta = theta + 360.0;
    theta = std::fmod(theta, 360.0);
  }
  double weightedAngleAvg(double w0, double rad0, double w1, double rad1) const {
    double avgCos = w0 * std::cos(degreeToRad(rad0)) + w1 * std::cos(degreeToRad(rad1));
    double avgSin = w0 * std::sin(degreeToRad(rad0)) + w1 * std::sin(degreeToRad(rad1));
    return radToDegree(std::atan2(avgSin, avgCos));
  }
  double handleLength_, regionOffset_, regionSize_, regionAngle_, alpha_, beta_;
  int intervalsWhenDrawingCurves_;
  int multiplicativeFactor_;
  double distanceEqualityThreshold_, orientationEqualityThreshold_;
};

} //end namespace podi_navigation_helpers
#endif
