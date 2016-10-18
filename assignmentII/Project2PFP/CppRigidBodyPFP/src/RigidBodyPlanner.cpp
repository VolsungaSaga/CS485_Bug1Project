#include "RigidBodyPlanner.hpp"

RigidBodyPlanner::RigidBodyPlanner(RigidBodySimulator * const simulator)
{
    m_simulator = simulator;   
}

RigidBodyPlanner::~RigidBodyPlanner(void)
{
    //do not delete m_simulator  
}


RigidBodyMove RigidBodyPlanner::ConfigurationMove(void)
    
{
    RigidBodyMove move;

//add your implementation

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


