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
    //Checks best location
    m_hit[0] = xGoalPos;
    m_hit[1] = yGoalPos;
    //Checks best time to go 
    //Curr Dist from goal
    double xMove;
    double yMove;
    //m_leave 0 == former loc for x; m_leave 1 == former loc for y
    //m_hit 0 == best x loc; m_hit 1 == best y loc

    if (amITooClose(sensor)) {
       //Pretty much go around the circle until you
       //reach back to where you were and then make a note as
       //to the best path possible
       printf("Print timesRep: %d\n",timesRep); 
       if (timesRep <= 0) {
           double senseMoveX = sensor.m_xmin - xCurr2;
           double senseMoveY = sensor.m_ymin - yCurr2;
           if (timesRep == -1) {
	       m_hit[0] = m_leave[0] = senseMoveX;
	       m_hit[1] = m_leave[1] = senseMoveY;
	       timesRep += 1;
	       /*Move perpendVec = getPerpendUnitVector(m_hit[0],m_hit[1]);
	       move = getStepVector(perpendVec.m_dx,perpendVec.m_dy);
	       return move;
	       */
           } 
           if (m_hit[0] > (xGoalPos - (sensor.m_xmin-xCurr2)))
               m_hit[0] = sensor.m_xmin-xCurr2;
           if (m_hit[1] > (yGoalPos - (sensor.m_ymin-yCurr2)))
               m_hit[1] = sensor.m_ymin-yCurr2;
           //perpendVector gives us a straight line
           Move perpendVec = getPerpendUnitVector(senseMoveX,senseMoveY); 
           move = getStepVector(perpendVec.m_dx,perpendVec.m_dy);
           //Checks to see if our current locations will ==
           //our former locations; NOTE: NEEDS WORK
           /*if (((sensor.m_xmin-xCurr2) == m_leave[0]) && (((sensor.m_ymin-yCurr2) == m_leave[1]))) {
               timesRep += 1;
           }*/
           printf("xForm: %1.2f, yForm: %1.2f, xCurr: %1.2f, yCurr: %1.2f\n",m_leave[0],m_leave[1],(sensor.m_xmin-xCurr2),(sensor.m_ymin-yCurr2));
       }
       else if (timesRep == 1){
           //Here, we make our bug head go to the best vals
           if ((move.m_dx == m_hit[0]) && (move.m_dy == m_hit[1])) {
               //move in a straight line essentially lol
               xMove = (xGoalPos - xCurr2);
               yMove = (yGoalPos - yCurr2);
               Move perpendVec = getPerpendUnitVector(xMove,yMove);
               move = getStepVector(perpendVec.m_dx, perpendVec.m_dy);
           }
           else {
              xMove = (sensor.m_xmin-xCurr2);
              yMove = (sensor.m_ymin-yCurr2);
              Move perpendVec = getPerpendUnitVector(xMove,yMove); 
              move = getStepVector(perpendVec.m_dx,perpendVec.m_dy);
           }
           return move;
       }
    }
    else {
        //move in a straight line essentially lol
        if (timesRep < 0) {
            xMove = (xGoalPos - xCurr2);
            yMove = (yGoalPos - yCurr2);
        }
        else {
            //This code is our circle
            xMove = sensor.m_xmin - xCurr2;
            yMove = sensor.m_ymin - yCurr2;
        }
        move = getStepVector(xMove, yMove);
    }

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
       


