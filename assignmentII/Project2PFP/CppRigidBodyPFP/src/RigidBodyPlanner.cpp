#include "RigidBodyPlanner.hpp"
#include <iostream>
RigidBodyPlanner::RigidBodyPlanner(RigidBodySimulator * const simulator)
{
    m_simulator = simulator;   
}

RigidBodyPlanner::~RigidBodyPlanner(void)
{
    //do not delete m_simulator  
}

const std::vector<double> RigidBodyPlanner::getDifferentialVector(double p_x, double p_y) const
{
  // calculate the attractive forces for x and y
  double x_att = 0;
  double y_att = 0;

  //x_att = p_x - m_simulator->GetGoalCenterX(); // x = robotx - goalx
  // y_att = p_y - m_simulator->GetGoalCenterY(); // y = roboty - goaly
  x_att = m_simulator->GetGoalCenterX() - p_x;
  y_att = m_simulator->GetGoalCenterY() - p_y;
  // these will hold the running sum
  double repulsiveX = 0;
  double repulsiveY = 0;

  // calculate the repulsive force for each obsticle.
  for (int j = 0; j < m_simulator->GetNrObstacles(); ++j)
  {
    // get the closest point on the obsticle to this control point
    Point closest = m_simulator->ClosestPointOnObstacle(j, p_x, p_y);

    // repulsive force is the sum over all obsticles and all points.
    repulsiveX = repulsiveX + (1/((closest.m_x - p_x)*(closest.m_x - p_x)));
    repulsiveY = repulsiveY + (1/((closest.m_x - p_x)*(closest.m_x - p_x)));
  }

  // since it's only length two it's okay to return a copy.
  std::vector<double> differential(2);
  differential[0] = (x_att - repulsiveX);
  differential[1] = (y_att - repulsiveY);

  // need to make a unit vector
  double magnitude = std::sqrt((differential[0] * differential[0]) + 
    (differential[1] * differential[1]));
  differential[0] /= magnitude;
  differential[1] /= magnitude;
  return differential;
}

RigidBodyMove RigidBodyPlanner::ConfigurationMove(void)
{

    RigidBodyMove move;
    if (m_simulator->HasRobotReachedGoal())
    {
      move.m_dx = 0;
      move.m_dy = 0;
      move.m_dtheta = 0;
      return move;
    }

    double controlX = 0;
    double controlY = 0;
    double theta = 0;

    std::vector<double> configSpace(3);
    std::vector<double> differentialVector(3);
    std::vector<double> moveAccumulator(3);

    const double* verticies = m_simulator->GetRobotVertices();
    std::cout << "Num vert " << m_simulator->GetNrRobotVertices() << std::endl;
    for (int i = 0; i < 2 * m_simulator->GetNrRobotVertices(); i+=2)
    {
      controlX = *(verticies + i);
      controlY = *(verticies + (i + 1));
      differentialVector = getDifferentialVector(controlX, controlY);
      
      std::cout << "control x_i" << controlX << i << std::endl; 
      std::cout << "control y_i " << controlY << i << std::endl; 
      theta = m_simulator->GetRobotTheta();
  
      worldSpaceToConfigSpace(configSpace, controlX, controlY, theta, 
        differentialVector[0], differentialVector[1], 0.01, 0.01);    

      move.m_dx += configSpace[0];
      move.m_dy += configSpace[1];
      move.m_dtheta += configSpace[2];

 //     std::cout << "Conf x " << configSpace[0] << std::endl; 
 //     std::cout << "Conf y " << configSpace[1] << std::endl; 
//      std::cout << "control x " << controlX << std::endl; 
//      std::cout << "control y " << controlY << std::endl; 
    }
   
//    std::cout << "Differential x" << differentialVector[0] << std::endl;
//    std::cout << "Differential y" << differentialVector[1] << std::endl;

    move.m_dx = move.m_dx; //* -1;
    move.m_dy = move.m_dy; //* -1;
    move.m_dtheta = move.m_dtheta; //* -1;
    return move;
}

//POINT ARITHMETIC FUNCTIONS

//These functions return a Point of form {x1 [op] x2, y1 [op] y2}, where
// (x1, y1) = Point1, (x2 , y2) = Point2, and [op] represents the operation being performed. 

Point pointAdd(Point p1, Point p2){
  Point newPoint = {(p1.m_x + p2.m_x), (p1.m_y + p2.m_y)};
  return newPoint;

}

Point pointSubtract(Point p1, Point p2){
  Point newPoint = {(p1.m_x - p2.m_x), (p1.m_y - p2.m_y)};
  return newPoint;
}

Point pointMultiply(Point p1, Point p2){
  Point newPoint = {(p1.m_x * p2.m_x), (p1.m_y * p2.m_y)};
  return newPoint;
}



// std::vector<vector> matrixAdd(std::vector<std::vector<double>> m1, std::vector<std::vector<double>> m2){}

// std::vector<vector> matrixMultiply(std::vector<std::vector<double>> m1, std::vector<std::vector<double>> m2){}


