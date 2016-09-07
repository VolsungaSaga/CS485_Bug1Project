#include "BugAlgorithms.hpp"

BugAlgorithms::BugAlgorithms(Simulator * const simulator)
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
    
    Move perpendVector = getPerpendVector(xDistObstacle, yDistObstacle);

    moveVector = getStepVector(perpendVector.m_dx, perpendVector.m_dy);
    
  }

  else{
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
  
    //add your implementation
    Move move ={0,0};
    
    return move;
}


//Am I too close to an obstacle - within a certain distance, in other words?
bool BugAlgorithms::amITooClose(Sensor sensor)
{
  if(sensor.m_dmin <= m_simulator->GetWhenToTurn()){
    return true;
  }

  return false;
}

//Calculates the left-turning vector for a given vector (like the nearest
//obstacle vector, for instance) in the following way:

//X_perp = -y
//Y_perp = x

//Returns a Move struct.
Move BugAlgorithms::getPerpendVector(double x, double y){  
  double newX = -y; 
  double newY = x;

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
       


