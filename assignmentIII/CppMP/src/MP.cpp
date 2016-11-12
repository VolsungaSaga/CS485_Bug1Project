#include "MP.hpp"
#include "PseudoRandom.hpp"
#include "MyTimer.hpp"
#include <cstring>
#include <cmath>

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
//your code

  //First, check if the end state is acceptable.
  //Acceptable end states are not colliding with an obstacle AND
  // are a certain minimum distance (we'll try robot radius) from vid.
  m_simulator->SetRobotState(sto);
  double distanceX = sto[0] - m_vertices[vid]->m_state[0];
  double distanceY = sto[1] - m_vertices[vid]->m_state[1];
  double distance = getMagnitude(distanceX, distanceY);
  
  //If end state unacceptable, return empty-handed, momentarily defeated.
  if(!m_simulator->IsValidState() && (distance < m_simulator->GetRobotRadius())){
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
      return;
    }

  }

  //After iterating along the line, we're reasonably certain that sto is a good state.
  m_simulator->SetRobotState(sto);
  Vertex* newVertex = new Vertex();
  newVertex->m_parent = vid;
  newVertex->m_state[0] = sto[0];
  newVertex->m_state[1] = sto[1];
  if(m_simulator->HasRobotReachedGoal()){
    newVertex->m_type = 1;
  }
  else{
    newVertex->m_type = 0; //Note: Fix this, should be 2 if in goal region.
  }
  newVertex->m_nchildren = 0;

  MotionPlanner::AddVertex(newVertex);
  


  

}

void MotionPlanner::ExtendRandom(void)
{
    Clock clk;
    StartTime(&clk);
    //printf("Got to samples[2] declaration");
    double samples[2];
    m_simulator->SampleState(samples);

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

     
    
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendEST(void)
{
    Clock clk;
    StartTime(&clk);

    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendMyApproach(void)
{
    Clock clk;
    StartTime(&clk);
 
//your code
    
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




