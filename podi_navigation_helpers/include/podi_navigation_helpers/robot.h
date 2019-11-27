#ifndef _ROBOT_H
#define _ROBOT_H

#include <cmath>
#include <tuple>
#include <vector>
#include <functional>

#include <geometry_msgs/Point.h>

namespace podi_navigation_helpers {

  class Robot {
      public:
          Robot() {};
          Robot(float handleLength);
          Robot(float handleLength, std::vector<geometry_msgs::Point> footprint,
            int maxV, int minV, int maxW, int minW, int minInPlaceW, int maxLinAcc, int maxRotAcc,
            double simTime, int linSimGranularity, int rotSimGranularity);
          std::vector<std::tuple<int, int, int, int, int, std::vector<std::tuple<int, int, int, int, int> > > > getValidMoves(
            int x, int y, int theta, int v, int w, int vDiscretization, int wDiscretization, int vSamples, int wSamples) const;
          std::tuple<int, int, int> getHandlePosition(int x, int y, int theta) const;
          std::tuple<float, float, float> getHandlePosition(float x, float y, float theta) const;
          float handleLength_;
          std::vector<geometry_msgs::Point> footprint_;
      private:
          int round(int n, int roundTo) const;
          // Convert degrees to radians
          float degreeToRad(int degree) const {
            // added roundTo because sometimes the degree to rad conversion
            // was not working correctly for like 90 or 180 deg
            float roundTo = 0.0000001;
            float rad = degree*M_PI/180;
            return ((int)std::round(rad/roundTo))*roundTo;
          };
          /**
           * copied from the local planenr's trajectory_planner.h
           * @brief  Compute x position based on velocity
           * @param  xi The current x position
           * @param  vx The current x velocity
           * @param  vy The current y velocity
           * @param  theta The current orientation
           * @param  dt The timestep to take
           * @return The new x position
           */
          inline double computeNewXPosition(double xi, double vx, double vy, double theta, double dt) const {
            return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
          }
          /**
           * copied from the local planenr's trajectory_planner.h
           * @brief  Compute y position based on velocity
           * @param  yi The current y position
           * @param  vx The current x velocity
           * @param  vy The current y velocity
           * @param  theta The current orientation
           * @param  dt The timestep to take
           * @return The new y position
           */
          inline double computeNewYPosition(double yi, double vx, double vy, double theta, double dt) const {
            return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
          }
          /**
           * copied from the local planenr's trajectory_planner.h
           * @brief  Compute orientation based on velocity
           * @param  thetai The current orientation
           * @param  vth The current theta velocity
           * @param  dt The timestep to take
           * @return The new orientation
           */
          inline double computeNewThetaPosition(double thetai, double vth, double dt) const {
            return thetai + vth * dt;
          }
          //compute velocity based on acceleration
          // copied from the local planenr's trajectory_planner.h
          /**
           * @brief  Compute velocity based on acceleration
           * @param vg The desired velocity, what we're accelerating up to
           * @param vi The current velocity
           * @param a_max An acceleration limit
           * @param  dt The timestep to take
           * @return The new velocity
           */
          inline double computeNewVelocity(double vg, double vi, double a_max, double dt, double timeRemaining) const {
            if (vg == vi) return vg;
            // NOTE (amal): The reason I do a constant acc (which is the same
            // thing the local planner does) is to get a curved trajectory
            // where the robot faces in the tangent direction to the motion
            // (i.e. circular arcs). allowing for changing acc does not account
            // for that. But on the flipside, the robot only has 3 possible
            // accelerations: a_max, -a_max, and 0. That doesn't seem too
            // realistic, so maybe I should think of better models of the
            // motion of a nonholonomic robot?
            double acc = a_max;//std::min((double)(abs((vg - vi) / timeRemaining)), a_max);//
            if ((vg - vi) > 0) {
              return std::min(vg, vi + acc * dt);
            }
            if ((vg - vi) < 0) {
               return std::max(vg, vi - acc * dt);
            }
          }

          int maxV_, minV_, maxW_, minW_, minInPlaceW_, maxLinAcc_, maxRotAcc_;
          int linSimGranularity_, rotSimGranularity_;
          double simTime_;
  };

} //end namespace podi_navigation_helpers
#endif
