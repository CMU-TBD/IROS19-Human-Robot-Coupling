#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>

#include <podi_navigation_helpers/couplingModel.h>

bool floatEqual(float x0, float x1) {
  float epsilon = 0.0001;
  if (fabs(x0-x1) < epsilon) return true;
  return false;
}

void testSetRegionAttributes(podi_navigation_helpers::CouplingModel* cm,
  float handleX, float handleY, float handleTheta, float robotV,
  float cxAns, float cyAns, float startingAngleAns, float regionAngleAns) {
  cm->setRegionAttributes(handleX, handleY, handleTheta, robotV);
  if (!floatEqual(cm->r_->cx, cxAns))
    ROS_ERROR("For robotV=%f, expected cx=%f, got cx=%f", robotV, cxAns, cm->r_->cx);
  if (!floatEqual(cm->r_->cy, cyAns))
    ROS_ERROR("For robotV=%f, expected cy=%f, got cy=%f", robotV, cyAns, cm->r_->cy);
  if (!floatEqual(cm->r_->startingAngle, startingAngleAns))
    ROS_ERROR("For robotV=%f, expected startingAngle=%f, got startingAngle=%f", robotV, startingAngleAns, cm->r_->startingAngle);
  if (!floatEqual(cm->r_->regionAngle, regionAngleAns))
    ROS_ERROR("For robotV=%f, expected regionAngle=%f, got regionAngle=%f", robotV, regionAngleAns, cm->r_->regionAngle);
}

void testClosestPointInRegion(podi_navigation_helpers::CouplingModel* cm,
  float handleTheta, std::tuple<float, float, float> p, std::tuple<float, float, float> ans) {
  std::tuple<float, float, float> retVal = cm->closestPointInRegion(p, handleTheta);
  if (!floatEqual(std::get<0>(retVal), std::get<0>(ans)))
    ROS_ERROR("Expected x=%f, got x=%f", std::get<0>(ans), std::get<0>(retVal));
  if (!floatEqual(std::get<1>(retVal), std::get<1>(ans)))
    ROS_ERROR("Expected y=%f, got y=%f", std::get<1>(ans), std::get<1>(retVal));
  if (!floatEqual(std::get<2>(retVal), std::get<2>(ans)))
    ROS_ERROR("Expected th=%f, got th=%f", std::get<2>(ans), std::get<2>(retVal));
}

void testGetNewTheta(podi_navigation_helpers::CouplingModel* cm,
  float handleX, float handleY, float humanX, float humanY, float humanTheta,
  float humanXNew, float humanYNew, float humanThetaNewAns) {
  float humanThetaNew = cm->getNewTheta(handleX, handleY, humanX, humanY,
    humanTheta, humanXNew, humanYNew);
  if (!floatEqual(humanThetaNew, humanThetaNewAns))
    ROS_ERROR("For handle=(%f, %f), human=(%f, %f, %f), humanNew=(%f, %f), expected %f but got %f",
    handleX, handleY, humanX, humanY, humanTheta, humanXNew, humanYNew, humanThetaNewAns, humanThetaNew);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "podi_test_grid");

  ros::NodeHandle n;

  // Test a couplingModel
  float minArmLength = 20;
  float averageArmLength = 40;
  float maxArmLength = 50;
  float maxV = 100;
  float minAngle = 10;
  float minV = 10;
  float maxAngle = 100;
  float thetaWeight = 1.0; // completely face towards the handle
  int intervalsWhenDrawingCurves = 0;
  int multiplicativeFactor = 1;
  podi_navigation_helpers::CouplingModel* cm = new podi_navigation_helpers::CouplingModel(
    minArmLength, averageArmLength, maxArmLength, maxV, minAngle,
    minV, maxAngle, thetaWeight, intervalsWhenDrawingCurves, multiplicativeFactor
  );

  ROS_WARN("Testing CouplingModel...");
  ROS_WARN("Testing CouplingModel::setRegionAttributes...");
  // test that regionAngle and startingAngle are correctly determined from velocity
  testSetRegionAttributes(cm, 0, 0, 0, 0, 0, 0, 130, 100);
  testSetRegionAttributes(cm, 0, 0, 0, 10, 0, 0, 130, 100);
  testSetRegionAttributes(cm, 0, 0, 0, 55, 0, 0, 152.5, 55);
  testSetRegionAttributes(cm, 0, 0, 0, 100, 0, 0, 175, 10);
  testSetRegionAttributes(cm, 0, 0, 0, 150, 0, 0, 175, 10);

  // test that startingAngle is correctly determined from theta
  testSetRegionAttributes(cm, 0, 0, 0, 0, 0, 0, 130, 100);
  testSetRegionAttributes(cm, 0, 0, 45, 10, 0, 0, 175, 100);
  testSetRegionAttributes(cm, 0, 0, 90, 55, 0, 0, 242.5, 55);
  testSetRegionAttributes(cm, 0, 0, 225, 100, 0, 0, 40, 10);
  testSetRegionAttributes(cm, 0, 0, 270, 150, 0, 0, 85, 10);
  testSetRegionAttributes(cm, 0, 0, 360, 150, 0, 0, 175, 10);
  testSetRegionAttributes(cm, 0, 0, -45, 150, 0, 0, 130, 10);

  // test that cx and cy are correctly determined from handleX and handleY
  testSetRegionAttributes(cm, 0, 0, 0, 0, 0, 0, 130, 100);
  testSetRegionAttributes(cm, 10, 0, 45, 10, 10, 0, 175, 100);
  testSetRegionAttributes(cm, 100, 0, 90, 55, 100, 0, 242.5, 55);
  testSetRegionAttributes(cm, 28, 37, 225, 100, 28, 37, 40, 10);
  testSetRegionAttributes(cm, -5, 67, 270, 150, -5, 67, 85, 10);
  testSetRegionAttributes(cm, 32, -999, 360, 150, 32, -999, 175, 10);
  testSetRegionAttributes(cm, -90, -70, -45, 150, -90, -70, 130, 10);
  ROS_WARN("Finished testing CouplingModel::setRegionAttributes...");

  // test closestPointInPlanner for points in the region
  float handleX = 0;
  float handleY = 0;
  float handleTheta = 0;
  float robotV = 0; // 100 degrees
  cm->setRegionAttributes(handleX, handleY, handleTheta, robotV);
  ROS_WARN("Testing CouplingModel::closestPointInRegion...");
  // Check points on the border of the region
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-50, 0, 0), std::make_tuple<float, float, float>(-50, 0, 0)); // point on large curve
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-32.13938, 38.30222, -10), std::make_tuple<float, float, float>(-32.13938, 38.30222, -10)); // point on corner of large curve and start line
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-19.28363, 22.98133, -27), std::make_tuple<float, float, float>(-19.28363, 22.98133, -27)); // point on start line
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-12.85575, 15.32089, -27), std::make_tuple<float, float, float>(-12.85575, 15.32089, -27)); // point on corner of small curve and start line
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-20, 0, 100), std::make_tuple<float, float, float>(-20, 0, 100)); // point on small curve
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-12.85575, -15.32089, -27), std::make_tuple<float, float, float>(-12.85575, -15.32089, -27)); // point on corner of small curve and end line
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-19.28363, -22.98133, -27), std::make_tuple<float, float, float>(-19.28363,- 22.98133, -27)); // point on end line
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-32.13938, -38.30222, -10), std::make_tuple<float, float, float>(-32.13938, -38.30222, -10)); // point on corner of large curve and end line

  // Check points inside the region
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-40, 0, 0), std::make_tuple<float, float, float>(-40, 0, 0));
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-20, 19, 0), std::make_tuple<float, float, float>(-20, 19, 0));
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-20, -19, 0), std::make_tuple<float, float, float>(-20, -19, 0));

  // test closestPointInPlanner for below the big curve
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-38.56726, 45.96267, -77), std::make_tuple<float, float, float>(-32.13938, 38.30222, -77)); // point along starting angle, but 60 units away
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-72.44444, 19.41143, -77), std::make_tuple<float, float, float>(-48.29629, 12.94095, -77)); // point between starting angle and angle that is in the middle of the region, 75 units away
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-53.5, 0, 0), std::make_tuple<float, float, float>(-50, 0, 0)); // point along central angle
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-98.76883, -15.64345, -90), std::make_tuple<float, float, float>(-49.38442, -7.82172, -90)); // point between ending angle and angle that is in the middle of the region, 100 units away
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-636.35973, -758.38400, -10), std::make_tuple<float, float, float>(-32.13938, -38.30222, -10)); // point along end line, but 990 units away

  // test closestPointInPlanner for points mapping to the small curve
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(0, 0, -77), std::make_tuple<float, float, float>(-20, 0, -77)); // cx, cy
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-0.64279, 0.76604, -27), std::make_tuple<float, float, float>(-12.85575, 15.32089, -27)); // 1 unit in the starting angle direction
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-0.76604, 0.64279, -27), std::make_tuple<float, float, float>(-15.32089, 12.85575, -27)); // 1 unit in 140 degrees
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-1, 0, -77), std::make_tuple<float, float, float>(-20, 0, -77)); // 1 unit in 180 degrees
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-0.74314, -0.66913, -27), std::make_tuple<float, float, float>(-14.86290, -13.38261, -27)); // 1 unit in 222 degrees
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-0.64279, -0.76604, -27), std::make_tuple<float, float, float>(-12.85575, -15.32089, -27)); // 1 unit in the ending angle direction
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-13.59462, 6.33927, -27), std::make_tuple<float, float, float>(-18.12616, 8.45237, -27)); // 15 units in 155 degrees
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-13.59462, -6.33927, -27), std::make_tuple<float, float, float>(-18.12616, -8.45237, -27)); // 15 unit in 205 degrees

  // test closestPointInPlanner for points beyond the starting line
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-38.56726, 45.96367, -77), std::make_tuple<float, float, float>(-32.13938, 38.30222, -77)); // point along starting line, 60 units away, shifted slightly up
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-31.37334, 38.94501, -10), std::make_tuple<float, float, float>(-32.13938, 38.30222, -10)); // point on corner of large curve and start line, plus one unit in the perpendicular direction
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-18.52258, 23.63008, -27), std::make_tuple<float, float, float>(-19.28862, 22.98729, -27)); // point on start line, plus one unit in the perpendicular direction
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-12.08971, 15.96368, -27), std::make_tuple<float, float, float>(-12.85575, 15.32089, -27)); // point on corner of small curve and start line, plus one unit in the perpendicular direction
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(64.27876, 76.60444, -77), std::make_tuple<float, float, float>(-12.85575, 15.32089, -77)); // point along ending line, 100 units in opposite direction
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(64.17876, 76.70444, -77), std::make_tuple<float, float, float>(-12.85575, 15.32089, -77)); // point along ending line, 100 units in opposite direction, shifted slightly up and left
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(64.37876, 76.50444, -77), std::make_tuple<float, float, float>(-12.85575, 15.32089, -77)); // point along ending line, 100 units in opposite direction, shifted slightl;y down and right
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-6.42788, 7.76044, -50), std::make_tuple<float, float, float>(-12.85575, 15.32089, -50)); // point halfway between cx,cy and minArmLength, along starting line, slightly shifted up
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(0, 100, -77), std::make_tuple<float, float, float>(-32.13938, 38.30222, -77)); // random point on the y axis
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(0, 75, -77), std::make_tuple<float, float, float>(-32.13938, 38.30222, -77)); // random point on the y axis
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(0, 50, -77), std::make_tuple<float, float, float>(-24.62019, 29.34120, -77)); // random point on the y axis
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(0, 35, -77), std::make_tuple<float, float, float>(-17.23414, 20.53884, -77)); // random point on the y axis
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(0, 25, -77), std::make_tuple<float, float, float>(-12.85575, 15.32089, -77)); // random point on the y axis
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(0, 12, -77), std::make_tuple<float, float, float>(-12.85575, 15.32089, -77)); // random point on the y axis
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(0, 1, -77), std::make_tuple<float, float, float>(-12.85575, 15.32089, -77)); // random point on the y axis
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(45.31539, 21.13091, -77), std::make_tuple<float, float, float>(-12.85575, 15.32089, -77)); // point 25 degrees, 50 units away
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(50, 0, -77), std::make_tuple<float, float, float>(-12.85575, 15.32089, -77)); // point on the x axis NOTE: points on the x-axis beyond cx can be arbitrarily mapped to the corner on starting line or the corner on ending line, currently I map them to the start, but perhaps alrenate based on a mod two to avoid bias / skewness? Practically it shouldn't matter though since the human won't be in front of the robot ever :O
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(1, 0, -77), std::make_tuple<float, float, float>(-12.85575, 15.32089, -77)); // point on the x axis

  // test closestPointInPlanner for points beyond the ending line
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-38.56726, -45.96367, -77), std::make_tuple<float, float, float>(-32.13938, -38.30222, -77)); // point along end line, 60 units away, shifted slightly down
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-31.37334, -38.94501, -10), std::make_tuple<float, float, float>(-32.13938, -38.30222, -10)); // point on corner of large curve and end line, plus one unit in the perpendicular direction
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-18.52258, -23.63008, -27), std::make_tuple<float, float, float>(-19.28862, -22.98729, -27)); // point on end line, plus one unit in the perpendicular direction
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-12.08971, -15.96368, -27), std::make_tuple<float, float, float>(-12.85575, -15.32089, -27)); // point on corner of small curve and end line, plus one unit in the perpendicular direction
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(64.27876, -76.60444, -77), std::make_tuple<float, float, float>(-12.85575, -15.32089, -77)); // point along starting line, 100 units in opposite direction
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(64.17876, -76.70444, -77), std::make_tuple<float, float, float>(-12.85575, -15.32089, -77)); // point along starting line, 100 units in opposite direction, shifted slightly down and right
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(64.37876, -76.50444, -77), std::make_tuple<float, float, float>(-12.85575, -15.32089, -77)); // point along starting line, 100 units in opposite direction, shifted slightly up and left
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-6.42788, -7.76044, -50), std::make_tuple<float, float, float>(-12.85575, -15.32089, -50)); // point halfway between cx,cy and minArmLength, along ending line, slightly shifted down
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(0, -100, -77), std::make_tuple<float, float, float>(-32.13938, -38.30222, -77)); // random point on the y axis
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(0, -75, -77), std::make_tuple<float, float, float>(-32.13938, -38.30222, -77)); // random point on the y axis
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(0, -50, -77), std::make_tuple<float, float, float>(-24.62019, -29.34120, -77)); // random point on the y axis
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(0, -35, -77), std::make_tuple<float, float, float>(-17.23414, -20.53884, -77)); // random point on the y axis
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(0, -25, -77), std::make_tuple<float, float, float>(-12.85575, -15.32089, -77)); // random point on the y axis
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(0, -12, -77), std::make_tuple<float, float, float>(-12.85575, -15.32089, -77)); // random point on the y axis
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(0, -1, -77), std::make_tuple<float, float, float>(-12.85575, -15.32089, -77)); // random point on the y axis
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(45.31539, -21.13091, -77), std::make_tuple<float, float, float>(-12.85575, -15.32089, -77)); // point -25 degrees, 50 units away

  // Change robotV, handleX, handleY, handleTheta, and test with that
  handleX = 20;
  handleY = -15;
  handleTheta = 45;
  robotV = 40; // 70 degrees
  cm->r_->maxRadius = 150;
  cm->r_->minRadius = 50;
  cm->setRegionAttributes(handleX, handleY, handleTheta, robotV);
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(20, -15, 0), std::make_tuple<float, float, float>(-15.35534, -50.35534, 0)); // cx, cy
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(2.32233, -32.67767, 0), std::make_tuple<float, float, float>(-15.35534, -50.35534, 0)); // central angle, 25 units in
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-15.35534, -50.35534, 0), std::make_tuple<float, float, float>(-15.35534, -50.35534, 0)); // center of small arc
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-4.62019, -19.34120, 0), std::make_tuple<float, float, float>(-29.24038, -23.68241, 0)); // along starting line, 25 units in
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-29.24038, -23.68241, 0), std::make_tuple<float, float, float>(-29.24038, -23.68241, 0)); // corner of starting line and small curve
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(15.65880, -39.62019, 0), std::make_tuple<float, float, float>(11.31759, -64.24039, 0)); // along ending line, 25 units in
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(11.31759, -64.24039, 0), std::make_tuple<float, float, float>(11.31759, -64.24039, 0)); // corner of ending line and small curve
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-8.19078, -25.26060, 0), std::make_tuple<float, float, float>(-26.98463, -32.10101, 0)); // 30 units in, 200 degrees
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(9.73940, -43.19078, 0), std::make_tuple<float, float, float>(2.89899, -61.98463, 0)); // 30 units in, 250 degrees
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-58.78462, -28.89185, 0), std::make_tuple<float, float, float>(-58.78462, -28.89185, 0)); // on startingLine, 80 units in
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-17.62222, -118.36619, 0), std::make_tuple<float, float, float>(-17.62222, -118.36619, 0)); // 110 units in, 250 degrees
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-6.04723, -162.72117, 0), std::make_tuple<float, float, float>(-6.04723, -162.72117, 0)); // corner of larger arc and ending line
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-214.92316, -100.50504, 0), std::make_tuple<float, float, float>(-120.95389, -66.30302, 0)); // 250 units in, 200 degrees
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-275.44233, -67.09445, 0), std::make_tuple<float, float, float>(-127.72117, -41.04723, 0)); // on starting line, 300 units in
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-145.08599, 57.43355, 0), std::make_tuple<float, float, float>(-127.72117, -41.04723, 0)); // 100 units away from the corner of the larger arc and starting angle, in a perpendicular direction
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(-67.46703, 20.34854, 0), std::make_tuple<float, float, float>(-58.78462, -28.89185, 0)); // on startingLine, 80 units in, plus 50 units out in a perpendicular direction
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(20.99619, -15.08716, 0), std::make_tuple<float, float, float>(11.31759, -64.24039, 0)); // 1 unit, 355 degrees from cx, cy
  testClosestPointInRegion(cm, handleTheta, std::make_tuple<float, float, float>(20, -115, 0), std::make_tuple<float, float, float>(2.89899, -111.98463, 0)); // 100 unit, 270 degrees from cx, cy
  ROS_WARN("Finished testing CouplingModel::closestPointInRegion...");

  // Test getNewTheta with thetaWeight = 0 (i.e. always face the handle)
  ROS_WARN("Testing CouplingModel::getNewTheta...");
  testGetNewTheta(cm, 0, 0,
                      100, 0, 0,
                      100, 0,
                      180);
  testGetNewTheta(cm, 0, 0,
                      100, 0, 50,
                      100, 0,
                      180);
  testGetNewTheta(cm, 0, 0,
                      102, -27, 0,
                      100, 10,
                      185.71059);
  testGetNewTheta(cm, -6, 2,
                      300, -500, -69,
                      57, 3,
                      180.90938);
  testGetNewTheta(cm, -6, 2,
                      -1, -1, -1,
                      57, 300,
                      258.06290);
  testGetNewTheta(cm, 0, 0,
                      -102, 0, 509,
                      100, 100,
                      225);
  testGetNewTheta(cm, 5, 7,
                      100, 0, 50,
                      25, -77,
                      103.39250);
  testGetNewTheta(cm, 0, 0,
                      100, 0, 0,
                      0, 100,
                      270);
  testGetNewTheta(cm, -27, 36,
                      27, 3, 50,
                      -5, -70,
                      101.72511);
  testGetNewTheta(cm, 301, 0,
                      102, -27, 0,
                      100, 10,
                      357.15181);
  testGetNewTheta(cm, -66, 32,
                      300, -500, -69,
                      57, 3,
                      166.73352);
  testGetNewTheta(cm, -76, 2,
                      -1, -1, -1,
                      57, 300,
                      245.94838);
  testGetNewTheta(cm, 20, 03,
                      -102, 0, 509,
                      100, 100,
                      230.48616);
  testGetNewTheta(cm, 25, 71,
                      100, 0, 57.12345,
                      100, 0,
                      136.56935);
  testGetNewTheta(cm, 50, -50,
                      -5, -5, 0,
                      0, 0,
                      315.0);

  cm->thetaWeight_ = 0.0; // Completely face tangent to motion
  testGetNewTheta(cm, 0, 0,
                      100, 0, 0,
                      0, 100,
                      135);
  testGetNewTheta(cm, -27, 36,
                      27, 3, 50,
                      -5, -70,
                      246.32946);
  testGetNewTheta(cm, 301, 0,
                      102, -27, 0,
                      100, 10,
                      93.09406);
  testGetNewTheta(cm, -66, 32,
                      300, -500, -69,
                      57, 3,
                      115.78525);
  testGetNewTheta(cm, -76, 2,
                      -1, -1, -1,
                      57, 300,
                      79.09329);
  testGetNewTheta(cm, 20, 03,
                      -102, 0, 509,
                      100, 100,
                      26.33769);
  testGetNewTheta(cm, 25, 71,
                      100, 0, 57.12345,
                      100, 0,
                      57.12345); // when you stay in the same place, keep the same orientation
  testGetNewTheta(cm, 50, -50,
                      -5, -5, 0,
                      0, 0,
                      45.0);

  cm->thetaWeight_ = 0.5; // Face the average of the tangent and direction to handle
  testGetNewTheta(cm, 0, 0,
                      100, 0, 0,
                      0, 100,
                      202.5);
  testGetNewTheta(cm, -27, 36,
                      27, 3, 50,
                      -5, -70,
                      174.02729);
  testGetNewTheta(cm, 301, 0,
                      102, -27, 0,
                      100, 10,
                      45.122936);
  testGetNewTheta(cm, -66, 32,
                      300, -500, -69,
                      57, 3,
                      141.25939);
  testGetNewTheta(cm, -76, 2,
                      -1, -1, -1,
                      57, 300,
                      162.52083);
  testGetNewTheta(cm, 20, 03,
                      -102, 0, 509,
                      100, 100,
                      308.411926);
  testGetNewTheta(cm, 25, 71,
                      100, 0, 57.12345,
                      100, 0,
                      96.84640);
  testGetNewTheta(cm, 50, -50,
                      -5, -5, 0,
                      0, 0,
                      0.0); // Tests that the coupling model averages angles correctly (i.e. the average of 315 and 45 should be 0, not 180)
  ROS_WARN("Finished testing CouplingModel::getNewTheta...");

  ROS_WARN("Finished testing CouplingModel...");
  return 0;
}

// }
