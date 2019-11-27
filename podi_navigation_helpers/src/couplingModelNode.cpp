#include <iostream>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <string>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cstdio>
#include <ctime>
#include <Eigen/Core>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <podi_navigation_helpers/couplingModel.h>
#include <podi_navigation_helpers/robot.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include "sensor_msgs/Joy.h"

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

namespace podi_navigation_helpers {

class CouplingModelNode
{
public:
  CouplingModelNode();
  ~CouplingModelNode();
private:
  ros::NodeHandle private_nh;
	std::thread updateCouplingLoopThread;
  double loop_freq;
	// Coupling Model
	podi_navigation_helpers::CouplingModel* couplingModel_;
	tf::TransformListener tf_;
	ros::Publisher humanPosePub, humanRegionPub, humanPoseInOdomFramePub;
  // Frame names
  std::string baseFrame, odomFrame, mapFrame;
	// Synchronization constants to let the user reset the human position
	ros::Subscriber resetStateSub;
	bool isOldHumanPosSet;
	std::mutex isOldHumanPosSetMutex;
	void updateCouplingLoop();
	void resetStateCB(const std_msgs::Empty &msg);
  double degreeToRad(int degree) const {
    return degree*M_PI/180.0;
  };
  double radToDegree(double rad) const {
    return rad*180.0/M_PI;
  };
};

CouplingModelNode::CouplingModelNode() {
  ros::NodeHandle private_nh("~");

	// Create the Coupling Model
  double handleLength, regionOffset, regionSize, regionAngle, alpha, beta, distanceEqualityThreshold, orientationEqualityThreshold;
  int multiplicativeFactor = 1, intervalsWhenDrawingCurves;
  private_nh.param("handleLengthForCouplingModel", handleLength, double(0.171477));
  private_nh.param("regionOffset", regionOffset, double(0.506822));
  private_nh.param("regionSize", regionSize, double(0.241369));
  private_nh.param("regionAngle", regionAngle, double(39.8526)); // 1.001948 rads
  private_nh.param("alpha", alpha, double(0.676452));
  private_nh.param("beta", beta, double(0.854187));
  private_nh.param("distanceEqualityThreshold", distanceEqualityThreshold, double(0.1));
  private_nh.param("orientationEqualityThreshold", orientationEqualityThreshold, double(1.0));
  private_nh.param("intervals_when_drawing_curves", intervalsWhenDrawingCurves, 10);
  couplingModel_ = new podi_navigation_helpers::CouplingModel(
    handleLength, regionOffset, regionSize, regionAngle, alpha, beta, distanceEqualityThreshold, orientationEqualityThreshold,
    intervalsWhenDrawingCurves, multiplicativeFactor);

  // Create the tf listener and odom helper, which will allow us to get robot pose
	tf::TransformListener tf_(ros::Duration(10));
  private_nh.param("update_human_pose_freq", loop_freq, 5.0); // Hz

  private_nh.param("baseFrame", baseFrame, std::string("/base_link"));
  private_nh.param("odomFrame", odomFrame, std::string("/odom"));
  private_nh.param("mapFrame", mapFrame, std::string("/map"));

  // Publishers for human position, handle position, and human region
	humanPosePub = private_nh.advertise<geometry_msgs::PoseStamped>("human_position", 1);
  humanPoseInOdomFramePub = private_nh.advertise<geometry_msgs::PoseStamped>("human_position_in_odom_frame", 1);
	humanRegionPub = private_nh.advertise<geometry_msgs::PolygonStamped>("human_region", 1);

  // Variables to allow the human pos to be reset to a default
	isOldHumanPosSet = false;
	resetStateSub = private_nh.subscribe("resetState", 1000, &CouplingModelNode::resetStateCB, this);

  // Start the thread that will keep updating the coupling model
	updateCouplingLoopThread = std::thread(&CouplingModelNode::updateCouplingLoop, this);
}

CouplingModelNode::~CouplingModelNode() {
  updateCouplingLoopThread.join();
  if (couplingModel_)
    delete couplingModel_;
}

void CouplingModelNode::updateCouplingLoop() {
  double oldRobotX, oldRobotY, oldRobotTheta;
  double oldHumanX, oldHumanY, oldHumanTheta;
  double newRobotX, newRobotY, newRobotTheta;
  double newHumanX, newHumanY, newHumanTheta;
  // ros::Rate doesn't work across threads, that's why I have to create it in
  // this function and not the initialization function
  std::tuple<double, double, double> humanPos;
  ros::Rate loop_rate(loop_freq);
	while (!ros::isShuttingDown()) {
		// transfrom the 0 vector in \base_link to \map
		tf::Stamped<tf::Pose> base_pose, robot_pose;
    base_pose.frame_id_ = baseFrame;
    base_pose.stamp_ = ros::Time::now();
    base_pose.setOrigin(tf::Vector3(0, 0, 0));
		base_pose.setRotation(tf::Quaternion(0, 0, 0, 1));

		try {
			tf_.waitForTransform(odomFrame, base_pose.frame_id_, base_pose.stamp_, ros::Duration(1.0));
			tf_.transformPose(odomFrame, base_pose, robot_pose);
    } catch (tf::TransformException ex) {
      ROS_WARN("CouplingModelNode::updateCouplingLoop %s", ex.what());
      loop_rate.sleep();
			continue;
    }

		// transform that robot location to x, y, theta
		newRobotX = robot_pose.getOrigin().getX();
		newRobotY = robot_pose.getOrigin().getY(); // mm
		newRobotTheta = radToDegree(tf::getYaw(robot_pose.getRotation()));

		// get the human position, depending on if this is the first iteration or not
    std::unique_lock<std::mutex> isOldHumanPosSetLock(isOldHumanPosSetMutex);
		if (!isOldHumanPosSet) {
      isOldHumanPosSet = true;
			isOldHumanPosSetLock.unlock();
			humanPos = couplingModel_->getInitialHumanPose(newRobotX, newRobotY, newRobotTheta);
		} else {
      isOldHumanPosSetLock.unlock();
			// get the handle and transform that to a human position
			humanPos = couplingModel_->getHumanPose(oldRobotX, oldRobotY, oldRobotTheta, oldHumanX, oldHumanY, oldHumanTheta, newRobotX, newRobotY, newRobotTheta);
		}
    newHumanX = std::get<0>(humanPos);
    newHumanY = std::get<1>(humanPos);
    newHumanTheta = std::get<2>(humanPos);

		// convert the human pose to a msg and publish it
		geometry_msgs::PoseStamped human_pose_msg;
		human_pose_msg.header.frame_id = odomFrame;
    human_pose_msg.header.stamp = robot_pose.stamp_;
    human_pose_msg.pose.position.x = newHumanX;
    human_pose_msg.pose.position.y = newHumanY;
    human_pose_msg.pose.position.z = 0.0;
    human_pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(degreeToRad(newHumanTheta));
		humanPoseInOdomFramePub.publish(human_pose_msg);

    // also publish the human pose in the map frame
    tf::Stamped<tf::Pose> humanPoseInOdomFrame, humanPoseInMapFrame;
    tf::poseStampedMsgToTF(human_pose_msg, humanPoseInOdomFrame);
    try {
			tf_.waitForTransform(mapFrame, humanPoseInOdomFrame.frame_id_, humanPoseInOdomFrame.stamp_, ros::Duration(1.0));
			tf_.transformPose(mapFrame, humanPoseInOdomFrame, humanPoseInMapFrame);
    } catch (tf::TransformException ex) {
      ROS_WARN("CouplingModelNode::updateCouplingLoop %s", ex.what());
      loop_rate.sleep();
			continue;
    }
    human_pose_msg.header.frame_id = mapFrame;
    human_pose_msg.header.stamp = humanPoseInMapFrame.stamp_;
    human_pose_msg.pose.position.x = humanPoseInMapFrame.getOrigin().getX();
    human_pose_msg.pose.position.y = humanPoseInMapFrame.getOrigin().getY();
    human_pose_msg.pose.position.z = 0.0;
    human_pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(humanPoseInMapFrame.getRotation()));
		humanPosePub.publish(human_pose_msg);

		// get and publish the region
    geometry_msgs::PolygonStamped region = couplingModel_->drawRegion(newRobotX, newRobotY, newRobotTheta, odomFrame);
    humanRegionPub.publish(region);

    oldRobotX = newRobotX;
    oldRobotY = newRobotY;
    oldRobotTheta = newRobotTheta;

		oldHumanX = newHumanX;
    oldHumanY = newHumanY;
    oldHumanTheta = newHumanTheta;

		loop_rate.sleep();
	}
}

void CouplingModelNode::resetStateCB(const std_msgs::Empty &msg) {
	std::unique_lock<std::mutex> isOldHumanPosSetLock(isOldHumanPosSetMutex);
	isOldHumanPosSet = false;
	isOldHumanPosSetLock.unlock();
}

} // end namespace

int main(int argc, char **argv) {
	ros::init(argc, argv, "coupling_model_node");
  podi_navigation_helpers::CouplingModelNode cmn;
	ros::spin();

	return 0;
}
