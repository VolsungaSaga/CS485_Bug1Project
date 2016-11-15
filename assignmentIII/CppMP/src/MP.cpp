#include "MP.hpp"
#include "PseudoRandom.hpp"
#include "MyTimer.hpp"
#include <cstring>
#include <cmath>
#include <iostream>

MotionPlanner::MotionPlanner(Simulator * const simulator)  
{
    m_simulator = simulator;   

    Vertex *vinit = new Vertex();

    vinit->m_parent   = -1;   
    vinit->m_nchildren= 0;    
    vinit->m_state[0] = m_simulator->GetRobotCenterX();
    vinit->m_state[1] = m_simulator->GetRobotCenterY();

    AddVertex(vinit);
    m_vidAtGoal = -1;
    m_totalSolveTime = 0;
}

MotionPlanner::~MotionPlanner(void)
{
    //do not delete m_simulator  

    const int n = m_vertices.size();
    for(int i = 0; i < n; ++i)
	delete m_vertices[i];
}


void MotionPlanner::ExtendTree(const int    vid, 
			       const double sto[])
{
  double robotX = m_vertices[vid]->m_state[0];
  double robotY = m_vertices[vid]->m_state[1];
  //First, check if the end state is acceptable.
  //Acceptable end states are not colliding with an obstacle AND
  // are a certain minimum distance (we'll try robot radius) from vid.
  m_simulator->SetRobotState(sto);

  //To avoid over using areas that are already explored, I shall filter out
  // sample points 
  for(int i = 0; i < m_vertices.size(); i++){
    double distanceX = sto[0] - m_vertices[i]->m_state[0];
    double distanceY = sto[1] - m_vertices[i]->m_state[1];
    double distance = getMagnitude(distanceX, distanceY);
    if(distance < m_simulator->GetRobotRadius())
    {
      return;
    }
  }
  //If end state unacceptable, return empty-handed, momentarily defeated.
  if(!m_simulator->IsValidState()){
    return;
  }

  double vidState[2] = {0,0};
  vidState[0] = m_vertices[vid]->m_state[0];
  vidState[1] = m_vertices[vid]->m_state[1];

  double line[] = {sto[0] - vidState[0], sto[1] - vidState[1]};
  double magnitudeLine = this->getMagnitude(line);
  int numStepsOnLine = floor(magnitudeLine/(m_simulator->GetDistOneStep()));

  //Iterate along the line, checking each of our steps for a misstep, as it were. Missteps
  // will have us fail completely, but that's okay.
  for(int i = 1; i <= numStepsOnLine; i++){
    double iStepVector[] = {0, 0};
    IthStepOnLine(vidState,sto,i, m_simulator->GetDistOneStep(), iStepVector);
    m_simulator->SetRobotState(iStepVector);
    if(!m_simulator->IsValidState()){
      m_simulator->SetRobotCenter(robotX, robotY);
      return;
    }

  }

  //After iterating along the line, we're reasonably certain that sto is a good state.
  Vertex* newVertex = new Vertex();
  newVertex->m_parent = vid;
  newVertex->m_state[0] = sto[0];
  newVertex->m_state[1] = sto[1];
  if (m_simulator->HasRobotReachedGoal()){
    newVertex->m_type = 2;
  }
  else{
    newVertex->m_type = 0; //Note: Fix this, should be 2 if in goal region.
  }
  newVertex->m_nchildren = 0;

  MotionPlanner::AddVertex(newVertex);
  m_simulator->SetRobotState(sto);
}

void MotionPlanner::ExtendRandom(void)
{
    Clock clk;
    StartTime(&clk);
    //printf("Got to samples[2] declaration");
    double samples[2];
    //Now to roll the dice: 9 for sampling goal point, 0-8 for random sample.
    int diceResult = random() % 10;
    if(diceResult != 9){
      m_simulator->SampleState(samples);
    }
    else{
      samples[0] = PseudoRandomUniformReal(m_simulator->GetGoalCenterX() - m_simulator->GetGoalRadius(),
					   m_simulator->GetGoalCenterX() + m_simulator->GetGoalRadius());
      samples[1] = PseudoRandomUniformReal(m_simulator->GetGoalCenterY() - m_simulator->GetGoalRadius(),
					   m_simulator->GetGoalCenterY() + m_simulator->GetGoalRadius());

    }

    
    long max = m_vertices.size();
    
    long randomVertex = 0;
    if (max != 0)
        randomVertex = random() %  max;

    //printf("Got to ExtendTree call, so look there!");
    ExtendTree(randomVertex, samples);

    m_totalSolveTime += ElapsedTime(&clk);
}

void MotionPlanner::ExtendRRT(void)
{
    Clock clk;
    StartTime(&clk);

    // Fill up the sample bucket
    double samples[2];
    m_simulator->SampleState(samples);

    // find the closest
    int closestVid = 0;

    // get the least distance from the samples
    double smallestDist = std::sqrt( (samples[0]-m_vertices[0]->m_state[0])*(samples[0]-m_vertices[0]->m_state[0]) + 
        (samples[1]-m_vertices[0]->m_state[1])*(samples[1]-m_vertices[0]->m_state[1]));

    for(int i=0; i<m_vertices.size(); ++i)
    {
        double xVertex = m_vertices[i]->m_state[0];
        double yVertex = m_vertices[i]->m_state[1];
        double currDist = std::sqrt( (samples[0]-xVertex)*(samples[0]-xVertex) + (samples[1]-yVertex)*(samples[1]-yVertex) );

        if(currDist < smallestDist)
        {
            closestVid = i;
            smallestDist = currDist;
        }
    }

    // pass the samples 
    ExtendTree(closestVid,samples);
         
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendEST(void)
{
    Clock clk;
    StartTime(&clk);

    // sample "bucket"
    double samples[2];
    
    // get a random sample
    m_simulator->SampleState(samples);

    double w = 0;
    for (unsigned i = 0; i < m_vertices.size(); i++)
    {
        w += ( 1.0 / (m_vertices[i]->m_nchildren * m_vertices[i]->m_nchildren)) ;
    }

    // get a random value for weight 0 to w
    double rand = PseudoRandomUniformReal(0, w);

    int vid = 0;
    w = 0;
    for (unsigned i = 0; i < m_vertices.size(); ++i)
    {
        w += ( 1.0 / (m_vertices[i]->m_nchildren * m_vertices[i]->m_nchildren)) ;
        if (w >= rand)
        {
            vid = i;
            break;
	}
    }

    // pass the samples to extend tree
    ExtendTree(vid,samples);

    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendMyApproach(void)
{
	Clock clk;
	StartTime(&clk);

    double samples[2];
    bool uniqueVertex = false;
    int closestVid = 0 ;
    double minDistance;
    double xDistanceSquare;
    double yDistanceSquare;
    double xMinDistanceSquare;
    double yMinDistanceSquare;
    int robotSize = (int)m_simulator->GetRobotRadius(); //* 2;
    unsigned int vertexCount = m_vertices.size();
    //printf("VertexCount:%d\n",vertexCount);
    // Loop through all verteces to make sure we aren't selecting a point too close to other verteces
    m_simulator->SampleState(samples);
    xMinDistanceSquare = pow(samples[0] - m_vertices[0]->m_state[0], 2);
    yMinDistanceSquare = pow(samples[1] - m_vertices[0]->m_state[1], 2);
    minDistance = sqrt(xMinDistanceSquare + yMinDistanceSquare);
    for (unsigned int i = 0; i < vertexCount; i++){
        xDistanceSquare = pow(m_simulator->GetGoalCenterX() - m_vertices[i]->m_state[0], 2);
        yDistanceSquare = pow(m_simulator->GetGoalCenterY() - m_vertices[i]->m_state[1], 2);
        double closeness = sqrt(xDistanceSquare + yDistanceSquare);
        // Pick the closest vertex to the goal and not a vertex that was selected the first time
        if (closeness < minDistance && closestVid != i){
            minDistance = closeness;
            closestVid = i;
            uniqueVertex = true;
        }
    }


    // Try to add the point from the vertex closest to the goal
    if (uniqueVertex) {
        //printf("IS UNIQUE AND PRETTY\n");
        ExtendTree(closestVid, samples);
    }
    else {
        double samples[2];
        //Now to roll the dice: 9 for sampling goal point, 0-8 for random sample.
        int diceResult = random() % 10;
        if(diceResult != 9){
            m_simulator->SampleState(samples);
        }
        else{
            samples[0] = PseudoRandomUniformReal(m_simulator->GetGoalCenterX() - m_simulator->GetGoalRadius(),
                                           m_simulator->GetGoalCenterX() + m_simulator->GetGoalRadius());
            samples[1] = PseudoRandomUniformReal(m_simulator->GetGoalCenterY() - m_simulator->GetGoalRadius(),
                                           m_simulator->GetGoalCenterY() + m_simulator->GetGoalRadius());
        }

        long max = m_vertices.size();

        long randomVertex = 0;
        if (max != 0)
            randomVertex = random() %  max;

        //printf("Got to ExtendTree call, so look there!");
        ExtendTree(randomVertex, samples);
    }


	m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::AddVertex(Vertex * const v)
{
    if(v->m_type == Vertex::TYPE_GOAL)
	m_vidAtGoal = m_vertices.size();
    m_vertices.push_back(v); 
    if(v->m_parent >= 0)
	(++m_vertices[v->m_parent]->m_nchildren);
}

void MotionPlanner::GetPathFromInitToGoal(std::vector<int> *path) const
{
    std::vector<int> rpath;
    
    rpath.clear();
    
    int i = m_vidAtGoal;
    do
    {
	rpath.push_back(i);
	i = m_vertices[i]->m_parent;	
    } 
    while(i >= 0);
    
    path->clear();
    for(int i = rpath.size() - 1; i >= 0; --i)
	path->push_back(rpath[i]);
}

//Calculates the magnitude of the given vector.
double MotionPlanner::getMagnitude(double x, double y){

  return sqrt((x * x)+(y * y));
}

//Calculates magnitude of a double array vector.

double MotionPlanner::getMagnitude(double vector[]){

  return sqrt((vector[0] * vector[0]) + (vector[1] * vector[1]));

    }

//Returns the point at the ith step on the line between config1 (a leaf of our tree) and config2 (a sample configuration), given
// a certain step size.
void MotionPlanner::IthStepOnLine(const double config1[], const double config2[], int i, double stepSizeParam, double* iStepVector){
  double x_config1 = config1[0];
  double y_config1 = config1[1];

  double x_config2 = config2[0];
  double y_config2 = config2[1];
  
  double stepSize = stepSizeParam;
  double magnitude = getMagnitude(config1[0] - config2[0], config1[1] - config2[1]);

  //The x value of the point at the ith step on the line between config1 and config2 =
  // (The x val of a vector of this step size) + (displacement from origin_x to config1_x)
  //It is naturally the same for the y values.
  
  double x_iStep = ((stepSize * i)/ magnitude) * (x_config2 - x_config1) + x_config1;
  double y_iStep = ((stepSize * i)/ magnitude) * (y_config2 - y_config1) + y_config1;

  iStepVector[0] = x_iStep;
  iStepVector[1] = y_iStep;
}




