#include "BugAlgorithms.hpp"
#include <iostream>
#include <math.h>
BugAlgorithms::BugAlgorithms(Simulator * const simulator) :
    m_closest(2),
    m_simulator(simulator),
    m_onM(false),
    m_Mode(false),
    m_hitBug2(false)
{
    //add your initialization of other variables
    //that you might have declared

    m_closest[0] = 0;
    m_closest[1] = 0;

    m_mode = STRAIGHT;
    m_hit[0] = m_hit[1] = HUGE_VAL;
    m_leave[0] = m_leave[1] = 0;
    m_best[0] = m_best[1] = 0;
    m_distLeaveToGoal = HUGE_VAL;    
    timesRep = -1;  //For bug1 and bug2
 
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

    //current positions of x and y for our bug
    double xCurr2 = m_simulator->GetRobotCenterX();
    double yCurr2 = m_simulator->GetRobotCenterY();
    //goal positions
    double xGoalPos = m_simulator->GetGoalCenterX();
    double yGoalPos = m_simulator->GetGoalCenterY();
    //Checks best time to go 
    //Curr Dist from goal
    double xMove;
    double yMove;
    //m_leave 0 == former loc for x; m_leave 1 == former loc for y
    //m_hit 0 == best x loc; m_hit 1 == best y loc
    //printf("m_best[0],m_best[1]:%1.2f,%1.2f\n",m_best[0],m_best[1]);
    if ((timesRep >= 4) || (timesRep < 0)) {
        //This code moves in a straight line
        if ((timesRep < 0) && (amITooClose(sensor))) {
            m_best[0] = 1000;
            m_best[1] = 1000;
            timesRep+=1;
        }
        else if ((timesRep >= 4) && (!amITooClose(sensor))) {
            timesRep = -1;
            //m_leave[0] = m_leave[1] = 0;
            //m_best[0] = m_best[1] = 1000;
        }
        xMove = (xGoalPos - xCurr2);
        yMove = (yGoalPos - yCurr2);
        move = getStepVector(xMove, yMove);

    }
    else {
        //This code is our circle 
        if (timesRep >= 4) {
            xMove = (xGoalPos - xCurr2);
            yMove = (yGoalPos - yCurr2);
            move = getStepVector(xMove, yMove);
            return move;
        }
        if (timesRep <= 1) {
            if (timesRep == 0) {
                m_best[0] = m_leave[0] = xCurr2;//sensor.m_xmin-xCurr2;
                m_best[1] = m_leave[1] = yCurr2;//sensor.m_ymin-yCurr2;
            }
            Move perpVec = getPerpendUnitVector(sensor.m_xmin-xCurr2,sensor.m_ymin-yCurr2);
            /*xMove = perpVec.m_dx;
            yMove = perpVec.m_dy;*/
            move = getStepVector(perpVec.m_dx, perpVec.m_dy);
            timesRep++;
            return move;
        }
        //For actual movement
        xMove = sensor.m_xmin - xCurr2;
        yMove = sensor.m_ymin - yCurr2;
        //double m_leavePlusX = m_leave[0]-.14;
        double m_leaveMinusX = m_leave[0]-.10;
        //checkBest checks the current distance to goal pos from our current best
        double checkBest = (m_simulator->GetDistanceFromLoopFin(xGoalPos, yGoalPos, m_best[0], m_best[1]));
        //checkCurr checks our current distance to goal pos from our current loc
        double checkLast = (m_simulator->GetDistanceFromLoopFin(xGoalPos, yGoalPos, m_leave[0], m_leave[1]));
        double checkCurr = (m_simulator->GetDistanceFromLoopFin(xGoalPos, yGoalPos, xCurr2, yCurr2));

        //Checks our sensor data
        double checkSenseCurr = (m_simulator->GetDistanceFromLoopFin(xGoalPos, yGoalPos, (sensor.m_xmin-xCurr2), (sensor.m_ymin-yCurr2)));
        double checkSenseLast = (m_simulator->GetDistanceFromLoopFin(xGoalPos, yGoalPos, m_leave[0], m_leave[1]));
        if ((checkBest > checkCurr) && (timesRep < 3)) {
            m_best[0] = xCurr2;
            m_best[1] = yCurr2;
        }
        /*sprite needs to have a radius*/
        /*Params can be easily changed; think vals are hardcoded tho*/
        //NEEDS A LOTTA WORK; checkCircleX and checkCircleY needs work, specifically
        if ((timesRep == 2) && (checkCurr >= checkLast) && (m_best[0] > m_leave[0]||m_best[0] < m_leave[0])) {
            if (m_simulator->ArePointsNear(xCurr2,yCurr2,m_leave[0],m_leave[1]))
              timesRep += 1;
        }
        //Current problem now is with timesRep and making sure that it's able to jump off to the goal
        /*if ((timesRep == 3) && (((m_best[0]-0.1) <= xCurr2) && (xCurr2 <= m_best[0])) && (((m_best[1]-0.5 <= yCurr2) && (yCurr2 <= m_best[1])))) {
            timesRep += 1;
            //printf("WE MADE IT FAM\n");
            //exit(1);
        }*/
        if ((timesRep == 3) && (checkCurr <= checkBest)) {
            timesRep += 1;
        }
        Move perpVec = getPerpendUnitVector(xMove,yMove);
        /*xMove = perpVec.m_dx;
        yMove = perpVec.m_dy;*/
        move = getStepVector(perpVec.m_dx, perpVec.m_dy);
        printf("CheckBest:%1.2f\nCheckCurr:%1.2f\nCheckLast:%1.2f\n",checkBest,checkCurr,checkLast);
     }

     //double g = m_simulator->GetDistanceFromLoopFin(xMove,yMove,m_leave[0],m_leave[1]);
     //printf("\n");
     //printf("DistLoopFin:%1.2f\n",g);
     printf("timesRep:%d\n",timesRep);
     printf("xLeave: %1.2f, yLeave: %1.2f\n",m_leave[0],m_leave[1]);
     printf(" xLeaveRangeMin: %1.2f, yLeaveRangeMin: %1.2f\n xLeaveRangeMax:%1.2f, yLeaveRangeMax:%1.2f\n",m_leave[0]-.50,m_leave[1]-0.5,m_leave[0]+0.5,m_leave[1]+0.5);
     printf("xBest: %1.2f, yBest: %1.2f\n",m_best[0],m_best[1]);
     printf(" xBestRangeMin:%1.2f, yBestRangeMin:%1.2f\n",m_best[0]-0.5,m_best[1]-0.5);
     printf("xCurr: %1.2f, yCurr: %1.2f\n\n",xCurr2,yCurr2); //(sensor.m_xmin-xCurr2),(sensor.m_ymin-yCurr2));
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

    double closestX = m_closest[0];
    double closestY = m_closest[1];
  
    //Variables to store future move values.
    Move moveVector;

    if (!m_Mode)
    {
        if(!amITooClose(sensor))   
        {
            double moveX = (xGoal - xCurr);
            double moveY = (yGoal - yCurr);
            moveVector = getStepVector(moveX, moveY);    
        }
        else
        {
            double distanceFromClosestToGoal = std::sqrt( ((xGoal - closestX) * (xGoal - closestX)) + 
                ((yGoal - closestY) * (yGoal - closestY)));
            double distanceFromBotToGoal = m_simulator->GetDistanceFromRobotToGoal();

            if (!m_hitBug2)
            {
                m_closest[0] = xCurr;
                m_closest[1] = yCurr;
                m_hitBug2 = true;
            }
            else if (distanceFromClosestToGoal >= distanceFromBotToGoal)
            {
                m_closest[0] = xCurr;
                m_closest[1] = yCurr;
            }
            m_Mode = true;
        } 
        
    }
    else
    {
	double distanceFromClosestToGoal = std::sqrt( ((xGoal - closestX) * (xGoal - closestX)) + 
	    ((yGoal - closestY) * (yGoal - closestY)));
	double distanceFromBotToGoal = m_simulator->GetDistanceFromRobotToGoal();
            
	if (distanceFromBotToGoal  < distanceFromClosestToGoal && onMVector(xCurr, yCurr))
	{
	    m_Mode = false;
            double moveX = (xGoal - xCurr);
            double moveY = (yGoal - yCurr);
            moveVector = getStepVector(moveX, moveY);    
	}
	else
	{
	    moveVector = follow(sensor, xCurr, yCurr);
	}
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

//X_perp = -y/|vector|
//Y_perp = x/|vector|

//Returns a Move struct; essentially goes in a straight line
Move BugAlgorithms::getPerpendUnitVector(double x, double y){
  //Do some vector arithmetic to find the unit vector, adapting magnitude
  //to our step size.

  double newX = -y/getMagnitude(-y,x);
  double newY = x/getMagnitude(-y,x);

  Move newVector = {newX, newY};

  return newVector;
}

//Calculates the magnitude of the given vector.
double BugAlgorithms::getMagnitude(double x, double y){

  return sqrt((x * x)+(y * y));
}

