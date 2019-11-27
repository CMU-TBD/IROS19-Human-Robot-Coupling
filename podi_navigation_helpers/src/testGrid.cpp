#include <vector>

#include <ros/ros.h>
#include <ros/console.h>

#include <podi_navigation_helpers/grid.h>


void test2DGrid(podi_navigation_helpers::Grid grid, int x, int y, int xAns, int yAns, unsigned int cellXAns, unsigned int cellYAns) {
  std::vector<int> pos{x, y};
  std::tuple<std::vector<int>, std::vector<unsigned int> > gridCell = grid.getGridCell(pos);
  if (std::get<0>(gridCell)[0] != xAns || std::get<0>(gridCell)[1] != yAns ||
      std::get<1>(gridCell)[0] != cellXAns || std::get<1>(gridCell)[1] != cellYAns) {
      ROS_ERROR("For pos=(%d, %d), expected ans=(%d, %d) and cell=(%d, %d), but got ans=(%d, %d) and cell=(%d, %d)",
                x, y, xAns, yAns, cellXAns, cellYAns, std::get<0>(gridCell)[0], std::get<0>(gridCell)[1], std::get<1>(gridCell)[0], std::get<1>(gridCell)[1]);
  }
}


void test3DGrid(podi_navigation_helpers::Grid grid, int x, int y, int z, int xAns, int yAns, int zAns, unsigned int cellXAns, unsigned int cellYAns, unsigned int cellZAns) {
  std::vector<int> pos{x, y, z};
  std::tuple<std::vector<int>, std::vector<unsigned int> > gridCell = grid.getGridCell(pos);
  if (std::get<0>(gridCell)[0] != xAns || std::get<0>(gridCell)[1] != yAns || std::get<0>(gridCell)[2] != zAns ||
      std::get<1>(gridCell)[0] != cellXAns || std::get<1>(gridCell)[1] != cellYAns || std::get<1>(gridCell)[2] != cellZAns) {
      ROS_ERROR("For pos=(%d, %d, %d), expected ans=(%d, %d, %d) and cell=(%d, %d, %d), but got ans=(%d, %d, %d) and cell=(%d, %d, %d)",
                x, y, z, xAns, yAns, zAns, cellXAns, cellYAns, cellZAns, std::get<0>(gridCell)[0], std::get<0>(gridCell)[1], std::get<0>(gridCell)[2], std::get<1>(gridCell)[0], std::get<1>(gridCell)[1], std::get<1>(gridCell)[2]);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "podi_test_grid");

  ros::NodeHandle n;

  // Test a grid
  std::vector<unsigned int> gridSize0 {27, 24};
  std::vector<unsigned int> gridSize1 {9, 6};
  std::vector<unsigned int> gridSize2 {3, 2};
  std::vector<std::vector<unsigned int> > gridSizes {gridSize0, gridSize1, gridSize2};
  std::vector<unsigned int> distanceThresholds {300, 100};
  std::vector<int> gridOrigin = {0, 0};
  podi_navigation_helpers::Grid grid0(gridSizes, distanceThresholds, gridOrigin);
  std::vector<int> goal0{500, 500}; // goalRepElem = (499, 501)
  std::vector<std::vector<int> > goals{goal0};
  grid0.setGoals(goals);

  ROS_WARN("Testing Grid...");
  // test (0, 0)
  test2DGrid(grid0, 0, 0, 13, 12, 27, 24);
  // test a point slightly more than 300 away, in a 225 degree direction counterclockwise from the positive x axis
  test2DGrid(grid0, 285, 287, 283, 276, 27, 24);
  // test a point slightly less than 300 away, in a 225 degree direction counterclockwise from the positive x axis
  test2DGrid(grid0, 286, 288, 283, 291, 9, 6);
  // test the goal point
  test2DGrid(grid0, 500, 500, 499, 501, 3, 2);
  // test the goal representative point
  test2DGrid(grid0, 499, 501, 499, 501, 3, 2);
  // test a point slightly more than 100 away, in a 225 degree direction counterclockwise from the positive x axis
  test2DGrid(grid0, 428, 430, 427, 429, 9, 6);
  // test a point slightly less than 100 away, in a 225 degree direction counterclockwise from the positive x axis
  test2DGrid(grid0, 429, 431, 427, 429, 9, 6);
  // test a point close to 100 away, on the edge on a 9x6 grid so it goes to a 3x2 grid
  test2DGrid(grid0, 430, 432, 430, 433, 3, 2);

  // Test the same grid with a goal of (0, 0)
  goal0 = std::vector<int>{0, 0}; // goalRepElem = (1, 1)
  goals = std::vector<std::vector<int> >{goal0};
  grid0.setGoals(goals);
  // test the goal point
  test2DGrid(grid0, 0, 0, 1, 1, 3, 2);
  // test the goal representative point
  test2DGrid(grid0, 1, 1, 1, 1, 3, 2);
  // test a negative point on the edge of a 3x2 cell
  test2DGrid(grid0, -3, -2, -4, -3, 3, 2);
  // test a negative representative point of a 3x2 cell
  test2DGrid(grid0, -1, -1, -1, -1, 3, 2);
  // test a point slightly more than 100 away, in a 225 degree direction counterclockwise from the positive x axis
  test2DGrid(grid0, -72, -72, -76, -75, 9, 6);
  // test a point slightly less than 100 away, in a 225 degree direction counterclockwise from the positive x axis
  test2DGrid(grid0, -71, -71, -70, -71, 3, 2);
  // test a point close to 100 away, on the edge on a 9x6 grid so it goes to a 3x2 grid
  test2DGrid(grid0, -70, -70, -70, -71, 3, 2);

  // Test Two Goals That Are Far Apart
  std::vector<int> goal1{300, 300}; // goalRepElem = (301, 301)
  goals.push_back(goal1);
  grid0.setGoals(goals);
  // test a point slightly more than 300 away, in a 225 degree direction counterclockwise from the positive x axis
  test2DGrid(grid0, 85, 87, 85, 87, 9, 6);
  // test a point slightly less than 300 away, in a 225 degree direction counterclockwise from the positive x axis
  test2DGrid(grid0, 86, 88, 85, 87, 9, 6);
  // test the goal point
  test2DGrid(grid0, 300, 300, 301, 301, 3, 2);
  // test the goal representative point
  test2DGrid(grid0, 301, 301, 301, 301, 3, 2);
  // test a point slightly more than 100 away, in a 225 degree direction counterclockwise from the positive x axis
  test2DGrid(grid0, 228, 230, 229, 231, 9, 6);
  // test a point slightly less than 100 away, in a 225 degree direction counterclockwise from the positive x axis
  test2DGrid(grid0, 229, 231, 229, 231, 9, 6);
  // test the goal point
  test2DGrid(grid0, 0, 0, 1, 1, 3, 2);
  // test the goal representative point
  test2DGrid(grid0, 1, 1, 1, 1, 3, 2);
  // test a negative point on the edge of a 3x2 cell
  test2DGrid(grid0, -3, -2, -4, -3, 3, 2);
  // test a negative representative point of a 3x2 cell
  test2DGrid(grid0, -1, -1, -1, -1, 3, 2);
  // test a point slightly more than 100 away, in a 225 degree direction counterclockwise from the positive x axis
  test2DGrid(grid0, -72, -72, -76, -75, 9, 6);
  // test a point slightly less than 100 away, in a 225 degree direction counterclockwise from the positive x axis
  test2DGrid(grid0, -71, -71, -70, -71, 3, 2);
  // test a point close to 100 away, on the edge on a 9x6 grid so it goes to a 3x2 grid
  test2DGrid(grid0, -70, -70, -70, -71, 3, 2);
  // test a point in-between the two goals
  test2DGrid(grid0, 150, 150, 148, 153, 9, 6);
  // test a point in-between the two goals
  test2DGrid(grid0, 150, 100, 148, 99, 9, 6);
  // test a point in-between the two goals
  test2DGrid(grid0, 70, 70, 70, 71, 3, 2);

  // Test another grid where distanceThreshold is the minimum amount
  gridSize0 = std::vector<unsigned int>{100, 100};
  gridSize1 = std::vector<unsigned int>{50, 50};
  gridSizes = std::vector<std::vector<unsigned int> >{gridSize0, gridSize1};
  distanceThresholds = std::vector<unsigned int>{142};
  grid0 = podi_navigation_helpers::Grid(gridSizes, distanceThresholds, gridOrigin);
  goal0 = std::vector<int>{200, -200}; // goalRepElem = (225, -225)
  goals = std::vector<std::vector<int> >{goal0};
  grid0.setGoals(goals);

  // test points close to the distance threshold, 45 degrees counterclockwise from the positive x axis
  test2DGrid(grid0, 326, -124, 350, -150, 100, 100);
  test2DGrid(grid0, 325, -125, 350, -150, 100, 100);
  test2DGrid(grid0, 324, -126, 350, -150, 100, 100);
  // test points close to the distance threshold, 90 degrees counterclockwise from the positive x axis
  test2DGrid(grid0, 225, -82, 250, -50, 100, 100);
  test2DGrid(grid0, 225, -83, 250, -50, 100, 100);
  test2DGrid(grid0, 225, -84, 250, -50, 100, 100);
  // test close by points
  test2DGrid(grid0, 199, -199, 175, -175, 50, 50);
  test2DGrid(grid0, 200, -200, 225, -225, 50, 50);
  test2DGrid(grid0, 201, -201, 225, -225, 50, 50);
  // test points along a corner of the 100x100 to 50x50 transition
  test2DGrid(grid0, 100, -100, 125, -125, 50, 50);
  test2DGrid(grid0, 101, -101, 125, -125, 50, 50);
  test2DGrid(grid0, 99, -99, 50, -50, 100, 100);
  test2DGrid(grid0, 99, -101, 50, -150, 100, 100);
  test2DGrid(grid0, 101, -99, 150, -50, 100, 100);
  // test points along a corner of the 100x100 to 50x50 transition
  test2DGrid(grid0, 300, -200, 325, -225, 50, 50);
  test2DGrid(grid0, 301, -199, 350, -150, 100, 100);
  test2DGrid(grid0, 301, -201, 325, -225, 50, 50);
  test2DGrid(grid0, 299, -201, 275, -225, 50, 50);
  test2DGrid(grid0, 299, -199, 275, -175, 50, 50);
  // test points along a corner of the 100x100 to 50x50 transition
  test2DGrid(grid0, 400, -300, 450, -350, 100, 100);
  test2DGrid(grid0, 399, -299, 375, -275, 50, 50);
  test2DGrid(grid0, 401, -299, 450, -250, 100, 100);
  test2DGrid(grid0, 401, -301, 450, -350, 100, 100);
  test2DGrid(grid0, 399, -301, 350, -350, 100, 100);
  // test points along a corner of the 100x100 to 50x50 transition
  test2DGrid(grid0, 200, -400, 250, -450, 100, 100);
  test2DGrid(grid0, 201, -399, 225, -375, 50, 50);
  test2DGrid(grid0, 201, -401, 250, -450, 100, 100);
  test2DGrid(grid0, 199, -399, 150, -350, 100, 100);
  test2DGrid(grid0, 199, -401, 150, -450, 100, 100);
  // test points along a corner of the 100x100 to 50x50 transition
  test2DGrid(grid0, 300, -300, 350, -350, 100, 100);
  test2DGrid(grid0, 301, -299, 325, -275, 50, 50);
  test2DGrid(grid0, 301, -301, 350, -350, 100, 100);
  test2DGrid(grid0, 299, -301, 275, -325, 50, 50);
  test2DGrid(grid0, 299, -299, 275, -275, 50, 50);
  // test points in the middle, where all cells are 50x50
  test2DGrid(grid0, 350, -250, 375, -275, 50, 50);
  test2DGrid(grid0, 349, -249, 325, -225, 50, 50);
  test2DGrid(grid0, 349, -251, 325, -275, 50, 50);
  test2DGrid(grid0, 351, -249, 375, -225, 50, 50);
  test2DGrid(grid0, 351, -251, 375, -275, 50, 50);

  // Test a 3D grid
  gridSize0 = std::vector<unsigned int>{27, 24, 500};
  gridSize1 = std::vector<unsigned int>{9, 6, 250};
  gridSize2 = std::vector<unsigned int> {3, 2, 2};
  gridSizes = std::vector<std::vector<unsigned int> >{gridSize0, gridSize1, gridSize2};
  distanceThresholds = std::vector<unsigned int>{300, 100};
  gridOrigin = {0, 0, 0};
  grid0 = podi_navigation_helpers::Grid(gridSizes, distanceThresholds, gridOrigin);
  goal0 = std::vector<int>{500, 500}; // goalRepElem = (499, 501)
  goals = std::vector<std::vector<int> >{goal0};
  grid0.setGoals(goals);

  // test (0, 0, 0)
  test3DGrid(grid0, 0, 0, 0, 13, 12, 250, 27, 24, 500);
  // test a point slightly more than 300 away, in a 225 degree direction counterclockwise from the positive x axis
  test3DGrid(grid0, 285, 287, 499, 283, 276, 250, 27, 24, 500);
  // test a point slightly less than 300 away, in a 225 degree direction counterclockwise from the positive x axis
  test3DGrid(grid0, 286, 288, 499, 283, 291, 375, 9, 6, 250);
  // test the goal point
  test3DGrid(grid0, 500, 500, 500, 499, 501, 501, 3, 2, 2);
  // test the goal representative point
  test3DGrid(grid0, 499, 501, -499, 499, 501, -499, 3, 2, 2);
  // test a point slightly more than 100 away, in a 225 degree direction counterclockwise from the positive x axis
  test3DGrid(grid0, 428, 430, -500, 427, 429, -625, 9, 6, 250);
  // test a point slightly less than 100 away, in a 225 degree direction counterclockwise from the positive x axis
  test3DGrid(grid0, 429, 431, 0, 427, 429, 125, 9, 6, 250);
  // test a point close to 100 away, on the edge on a 9x6 grid so it goes to a 3x2 grid
  test3DGrid(grid0, 430, 432, 0, 430, 433, 1, 3, 2, 2);

  ROS_WARN("Finished testing Grid...");
  return 0;
}

// }
