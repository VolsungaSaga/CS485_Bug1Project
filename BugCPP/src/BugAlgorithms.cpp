#include "BugAlgorithms.hpp"

BugAlgorithms::BugAlgorithms(Simulator * const simulator)
{
    m_simulator = simulator;   
    //add your initialization of other variables
    //that you might have declared
    
    //I know that this has to be added otherwise it won't work
    simulator->ReadObstacles();
    
    m_mode = STRAIGHT;
    m_hit[0] = m_hit[1] = HUGE_VAL;
    m_leave[0] = m_leave[1] = 0;
    m_distLeaveToGoal = HUGE_VAL;    
}

BugAlgorithms::~BugAlgorithms(void)
{
    //do not delete m_simulator
}

//Made to cut down on code
double moveMore(double goal, double curr) {
    return (goal - curr)/m_simulator->GetDistanceFromRobotToGoal();
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
  double moveX;
  double moveY;


  if(amITooClose(sensor)){
    //Perpendicular vector of (a,b) = (-b, a) (for left turn)
    
    double xDistObstacle = sensor.m_xmin - xCurr;
    double yDistObstacle = sensor.m_ymin - yCurr;
    Move perpendVector = getPerpendUnitVector(xDistObstacle, yDistObstacle);
    moveX = perpendVector.m_dx;
    moveY = perpendVector.m_dy;
    
  }

  else{
    moveX = moveMore(xGoal,xCurr);
    moveY = moveMore(yGoal,yCurr);
  }

  Move move ={moveX,moveY};
  

  return move;
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
    //Former location and times
    double xFormLoc;
    double yFormLoc;
    //Checks best location
    double xBest = xGoalPos;
    double yBest = yGoalPos;
    //Checks best time to go
    int timesRep = 0;
    //Curr Dist from goal
    double checkX = |xGoalPos| - |xCurr2|;
    double checkY = |yGoalPos| - |yCurr2|;

    if (amITooClose(sensor)) {
       //Pretty much go around the circle until you
       //reach back to where you were and then make a note as
       //to the best path possible
       xBest = xFormLoc = xCurr2;
       yBest = yFormLoc = yCurr2;
       if (timesRep == 0) {
           move[0] = sensor.m_xmin - xCurr;
           move[1] = sensor.m_ymin - yCurr;
           if (checkX > (|xGoalPos| - |xCurr2|)
               xBest = xCurr2Pos;
           if (checkY > (|yGoalPos| - |yCurr2|))
               yBest = yCurr2Pos;
           checkX = |xGoalPos| - |xCurr2|;
           checkY = |yGoalPos| - |yCurr2|;
           //Checks to see if our current locations will ==
           //our former locations
           if ((xCurr2 == xFormLoc) && (yCurr2 == yFormLoc))
               timesRep = 1;

           return move;
       }
       else {
           //Here, we make our bug head go to the best vals
           if ((move[0] <= 0) && (move[1] <= 0)) {
              moveMore(xGoalPos,xCurr2); 
              moveMore(yGoalPos,yCurr2);
           }
           move[0] = xBest-m_xmin;
           move[1] = yBest-sensor.m_ymin;
       }
    }
    else {
       //move in a straight line essentially lol
       moveX = moveMore(xGoalPos,xCurr2);
       moveY = moveMore(yGoalPos,yCurr2);
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

//Calculates the left-turning vector for a given vector (like the nearest
//obstacle vector, for instance) in the following way:

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
       
//From the given x and y components of a vector, calculate the
// vector of magnitude 0.06 that
// goes in the same direction. That is:

// U = ((L^2)/|V|) * V , U = vector of magnitude L, V = given vector.


Move BugAlgorithms::getStepVector(double x, double y){
  double stepSize = (m_simulator->GetStep());
  double originalMagnitude = getMagnitude(x,y);

  double stepX = (stepSize/originalMagnitude) * x;
  double stepY = (stepSize/originalMagnitude) * y;

  Move stepVector = {stepX, stepY};

  return stepVector;


}

