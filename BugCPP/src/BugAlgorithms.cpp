#include "BugAlgorithms.hpp"
#include <iostream>
BugAlgorithms::BugAlgorithms(Simulator * const simulator) :
    m_goalX(simulator->GetGoalCenterX()), 
    m_goalY(simulator->GetGoalCenterY()),
    m_robotX(0),
    m_robotY(0)
{
    m_simulator = simulator;   
    //add your initialization of other variables
    //that you might have declared

    m_mode = STRAIGHT;
    m_hit[0] = m_hit[1] = HUGE_VAL;
    m_leave[0] = m_leave[1] = 0;
    m_distLeaveToGoal = HUGE_VAL;    
}

BugAlgorithms::~BugAlgorithms(void)
{
    //do not delete m_simulator  
}

Move BugAlgorithms::Bug0(Sensor sensor)
{
    //add your implementation
                
    m_goalX = m_simulator->GetGoalCenterX(); 
    m_goalY = m_simulator->GetGoalCenterY();
    m_robotX = m_simulator->GetRobotCenterX();
    m_robotY = m_simulator->GetRobotCenterY();

    Move move = {0,0};
    moveToGoal(move);
    std::cout << "Goal x " << m_goalX << " goal y " << m_goalY << std::endl;
    return move;
}

Move BugAlgorithms::Bug1(Sensor sensor)
{
    //add your implementation
    Move move ={0,0};

    return move;
}

Move BugAlgorithms::Bug2(Sensor sensor)
{
std::cout << "Hello from bug 2" << std::endl;
    //add your implementation
    Move move ={0,0};
    
    return move;
}

void BugAlgorithms::moveToGoal(Move& p_move)
{
    int x = 0;
    int y = 0;
    if (m_goalX < m_robotX)
    {
        p_move.m_dx = -0.01;
    }
    else if (m_goalX > m_robotX)
    {
        p_move.m_dx = 0.01;
    }
    else // it is 0 and we are on trajectory to goal
    {
        p_move.m_dx = 0;
    }

    if (m_goalY < m_robotY)
    {
        p_move.m_dy = -0.01;
    }
    else if (m_goalY > m_robotY)
    {
        p_move.m_dy = 0.01;
    }
    else
    {
        p_move.m_dy = 0;
    }
}       


