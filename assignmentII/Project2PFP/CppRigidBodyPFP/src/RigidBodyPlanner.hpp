#ifndef RIGID_BODY_PLANNER_HPP_
#define RIGID_BODY_PLANNER_HPP_

#include "RigidBodySimulator.hpp"

struct RigidBodyMove
{
  double m_dx;
  double m_dy;
  double m_dtheta;
};

class RigidBodyPlanner
{
public:
  RigidBodyPlanner(RigidBodySimulator * const simulator);
  
  ~RigidBodyPlanner(void);

  /*
   * This is the function that you should implement.
   * This function needs to compute by how much the position (dx, dy) 
   * and orientation (dtheta) should change so that the robot makes a small 
   * move toward the goal while avoiding obstacles, 
   * as guided by the potential field.
   *
   * You have access to the simulator.
   * You can use the methods available in simulator to get all the information
   * you need to correctly implement this function
   *
   */
  RigidBodyMove ConfigurationMove(void);


  //Helper Functions - Mostly having to do with point and matrix arithmetic.

  Point pointAdd(Point p1, Point p2);
  Point pointSubtract(Point p1, Point p2);
  Point pointMultiply(Point p1, Point p2);

  // std::vector<std::vector<double>> matrixAdd(std::vector<std::vector<double>> m1, std::vector<std::vector<double>> m2);
  // std::vector<std::vector<double>> matrixMultiply(std::vector<vector> m1, std::vector<vector> m2);
    
protected:
  RigidBodySimulator *m_simulator;
};

#endif
