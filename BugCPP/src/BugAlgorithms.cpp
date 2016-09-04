#include "BugAlgorithms.hpp"
#include <iostream>

#define MOVE_SIZE 0.01

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
    moveToGoal(move, sensor);
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

void BugAlgorithms::moveToGoal(Move& p_move, const Sensor& p_sensor)
{
    int x = 0;
    int y = 0;

    if (!hitObsticleHorizontal(p_sensor) && !hitObsticleVerticle(p_sensor))
    {
        if (m_goalX < m_robotX)
        {
	    p_move.m_dx = -MOVE_SIZE;
        }
        else if (m_goalX > m_robotX)
        {
    	    p_move.m_dx = MOVE_SIZE;
        }
        else // it is 0 and we are on trajectory to goal
        {
	    p_move.m_dx = 0;
        }

        if (m_goalY < m_robotY)
        {
	    p_move.m_dy = -MOVE_SIZE;
        }
        else if (m_goalY > m_robotY)
        {
            p_move.m_dy = MOVE_SIZE;
        }
        else
        {
	    p_move.m_dy = 0;
        }
    }
    else // we have an obsticle in front of us.
    {
        // we will move up try to move up first means we
        // had to hit horizontally
        if (hitObsticleHorizontal(p_sensor))
        {
            // decide which direction to move and move that way
        }
        // we have to move horizontally since we hit vertically.
        else if (hitObsticleVerticle(p_sensor))
        {
            // decide which direction to move and move that way
        }
    }
}       

//! TODO Finish horizontal and verticle functions sho
const bool BugAlgorithms::hitObsticleHorizontal(const Sensor& p_sensor) const
{
    // if the dmin is violated and the robot if a MOVE_SIZE from the obsticle
    if (p_sensor.m_dmin <= MOVE_SIZE && (m_robotX ) ) // << how to know if robot hit vertically
    {
        return true;
    }
    return false;
}

const bool BugAlgorithms::hitObsticleVerticle(const Sensor& p_sensor) const
{
    // stub
    return false;
}
