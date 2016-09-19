#include "BugAlgorithms.hpp"
#include <cmath>

BugAlgorithms::BugAlgorithms(Simulator * const simulator)
{
    m_simulator = simulator;   
    //add your initialization of other variables
    //that you might have declared
    
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
  //printf("HUGE_VAL:%1.2f,%1.2f\n",m_best[0],m_best[1]);
  if(amITooClose(sensor)){
    //Perpendicular vector of (a,b) = (-b, a) (for left turn)
    
    double xDistObstacle = sensor.m_xmin - xCurr;
    double yDistObstacle = sensor.m_ymin - yCurr;
    Move perpendVector = getPerpendUnitVector(xDistObstacle, yDistObstacle);
    moveVector = getStepVector(perpendVector.m_dx,perpendVector.m_dy);
  }

  else{
    double moveX = (xGoal-xCurr);
    double moveY = (yGoal-yCurr);
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
        else if (timesRep >= 4) {
            timesRep = -1;
            m_leave[0] = m_leave[1] = 0;
            //m_best[0] = m_best[1] = 1000;
        }
        xMove = (xGoalPos - xCurr2);
        yMove = (yGoalPos - yCurr2);
        move = getStepVector(xMove, yMove);
    }
    else {
        //This code is our circle 
        /*printf("m_leave[0]:%1.2f\n",m_leave[0]);
        printf("m_leave[1]:%1.2f\n",m_leave[1]);
        if (m_leave[0] == 0)
            exit(1);*/
        //exit(1);
        if (timesRep <= 1) {
            if (timesRep == 0) {
                m_leave[0] = sensor.m_xmin-xCurr2;
                m_leave[1] = sensor.m_ymin-yCurr2;
            }
            Move perpVec = getPerpendUnitVector(sensor.m_xmin-xCurr2,sensor.m_ymin-yCurr2);
            /*xMove = perpVec.m_dx;
            yMove = perpVec.m_dy;*/
            move = getStepVector(perpVec.m_dx, perpVec.m_dy);
            timesRep++;
            return move;
        }
        xMove = sensor.m_xmin - xCurr2;
        yMove = sensor.m_ymin - yCurr2;
        if (((xGoalPos - m_best[0]) < (xGoalPos - (sensor.m_xmin-xCurr2))) && (timesRep < 4))
            m_best[0] = sensor.m_xmin-xCurr2;
        if (((yGoalPos - m_best[1]) < (yGoalPos - (sensor.m_ymin-yCurr2))) && (timesRep < 4))
            m_best[1] = sensor.m_ymin-yCurr2;
        /*sprite needs to have a radius*/
        /*Params can be easily changed*/
        if ((timesRep == 2) && ((m_simulator->GetDistanceFromLoopFin(xMove, yMove, m_leave[0], m_leave[1])) >= 2.0)) {
            timesRep += 1;
            //exit(1);
        }
        if ((timesRep == 3) && ((m_simulator->GetDistanceFromLoopFin(xMove,yMove,m_best[0],m_best[1])) >= 0.25))  {
            timesRep += 1;
            //exit(1);
        }
        Move perpVec = getPerpendUnitVector(xMove,yMove);
        /*xMove = perpVec.m_dx;
        yMove = perpVec.m_dy;*/
        move = getStepVector(perpVec.m_dx, perpVec.m_dy);
     }

     double g = m_simulator->GetDistanceFromLoopFin(xMove,yMove,m_leave[0],m_leave[1]);
     printf("\n");
     printf("DistLoopFin:%1.2f\n",g);
     printf("timesRep:%d\n",timesRep);
     printf("xLeave: %1.2f, yLeave: %1.2f\n",m_leave[0],m_leave[1]);
     printf("xBest: %1.2f, yBest: %1.2f\n",m_best[0],m_best[1]);
     printf("xCurr: %1.2f, yCurr: %1.2f\n\n",(sensor.m_xmin-xCurr2),(sensor.m_ymin-yCurr2));

    return move;
}

Move BugAlgorithms::Bug2(Sensor sensor)
{
  
    //add your implementation
    Move move ={0,0};
    
    return move;
}



bool BugAlgorithms::amITooClose(Sensor sensor)
{
  if(sensor.m_dmin <= m_simulator->GetWhenToTurn()){
    return true;
  }

  return false;
}

//Code for getCircleVector experimental; going to circle around an obj
//Move BugAlgorithms::getCircleVector(double x, double y) {}

//Calculates the left-turning vector for a given vector (like the nearest
//obstacle vector, for instance) in the following way:

Move BugAlgorithms::getStepVector(double x, double y){
  double stepSize = (m_simulator->GetStep());
  double originalMagnitude = getMagnitude(x,y);

  double stepX = (stepSize/originalMagnitude) * x;
  double stepY = (stepSize/originalMagnitude) * y;

  Move stepVector = {stepX, stepY};

  return stepVector;


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
       


