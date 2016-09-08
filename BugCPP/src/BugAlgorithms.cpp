#include "BugAlgorithms.hpp"
#include <iostream>
BugAlgorithms::BugAlgorithms(Simulator * const simulator) :
    m_simulator(simulator),
    m_onM(false)
{
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
    //First, get my position.
    double xCurr = m_simulator->GetRobotCenterX();
    double yCurr = m_simulator->GetRobotCenterY();

    //The goal's position.
    double xGoal = m_simulator->GetGoalCenterX();
    double yGoal = m_simulator->GetGoalCenterY();

    //Variables to store future move values.
    Move moveVector;

    if(amITooClose(sensor))
    {
        moveVector = follow(sensor, xCurr, yCurr);
    }
    else
    {
        double moveX = (xGoal - xCurr);
        double moveY = (yGoal - yCurr);
        moveVector = getStepVector(moveX, moveY);
    }
    return moveVector;
}

Move BugAlgorithms::Bug1(Sensor sensor)
{
    //add your implementation
    Move move ={0,0};

    return move;
}

Move BugAlgorithms::Bug2(Sensor sensor)
{
    // does what bug 0 does except for when it hits an obsticle.
    double xCurr = m_simulator->GetRobotCenterX();
    double yCurr = m_simulator->GetRobotCenterY();

    //The goal's position.
    double xGoal = m_simulator->GetGoalCenterX();
    double yGoal = m_simulator->GetGoalCenterY();

    //Variables to store future move values.
    Move moveVector;

    if(amITooClose(sensor) && !m_onM)
    {
        moveVector = follow(sensor, xCurr, yCurr);
        
        if (onMVector(xCurr, yCurr))
        {
            m_onM = true;
        }
    }
    else
    {
        double moveX = (xGoal - xCurr);
        double moveY = (yGoal - yCurr);
        moveVector = getStepVector(moveX, moveY);
        m_onM = false;
    }
    return moveVector;
}


//Am I too close to an obstacle - within a certain distance, in other words?
const bool BugAlgorithms::amITooClose(const Sensor& sensor) const
{
    if(sensor.m_dmin <= m_simulator->GetWhenToTurn())
        return true;

    return false;
}

//Calculates the left-turning vector for a given vector (like the nearest
//obstacle vector, for instance) in the following way:

//X_perp = -y
//Y_perp = x

//Returns a Move struct.
Move BugAlgorithms::getPerpendVector(const double x, const double y) const
{  
    double newX = -y; 
    double newY = x;

    Move newVector = {newX, newY};

    return newVector;
}

//Calculates the magnitude of the given vector.
const double BugAlgorithms::getMagnitude(const double x, const double y) const
{
    return sqrt((x * x)+(y * y));
}

//From the given x and y components of a vector, calculate the
// vector of magnitude 0.06 that
// goes in the same direction. That is:

// U = ((L^2)/|V|) * V , U = vector of magnitude L, V = given vector.
Move BugAlgorithms::getStepVector(const double x, const double y) const
{
    double stepSize = (m_simulator->GetStep());
    double originalMagnitude = getMagnitude(x,y);

    double stepX = (stepSize/originalMagnitude) * x;
    double stepY = (stepSize/originalMagnitude) * y;

    Move stepVector = {stepX, stepY};

    return stepVector;
}
       
const bool BugAlgorithms::onMVector(const double x, const double y) const
{
    return m_simulator->IsPointNearLine(x, y, m_simulator->GetRobotInitX(), m_simulator->GetRobotInitY(), 
        m_simulator->GetGoalCenterX(), m_simulator->GetGoalCenterY());
}

Move BugAlgorithms::follow(const Sensor& sensor, double xCurr, double yCurr) const
{
    //Perpendicular vector of (a,b) = (-b, a) (for left turn)
    Move moveVector;
   
    double xDistObstacle = sensor.m_xmin - xCurr;
    double yDistObstacle = sensor.m_ymin - yCurr;
    
    Move perpendVector = getPerpendVector(xDistObstacle, yDistObstacle);
    moveVector = getStepVector(perpendVector.m_dx, perpendVector.m_dy);
    return moveVector;
}

