#include "MP.hpp"
#include "PseudoRandom.hpp"
#include "MyTimer.hpp"
#include <cstring>

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
  m_simulator->SetRobotState(sto);
  //If end state unacceptable, return empty-handed, momentarily defeated.
  if(!m_simulator->IsValidState()){
    return;
  }

  
  double[] vidState = m_vertices[vid].m_state;

  double[] line = {sto[0] - vidState[0], sto[1] - vidState[1]};
  double magnitudeLine = getMagnitude(line);
  int numStepsOnLine = Math.floor(magnitudeLine/(m_simulator->GetDistOneStep()));

  //Iterate along the line, checking each of our steps for a misstep, as it were. Missteps
  // will have us fail completely, but that's okay.
  for(int i = 1; i <= numStepsOnLine; i++){
    m_simulator->SetRobotState(IthStepOnLine(vidState,sto,i));
    if(!m_simulator->IsValidState()){
      return;
    }

  }

  //After iterating along the line, we're reasonably certain that sto is a good state.

  Vertex newVertex = new Vertex();
  newVertex->m_parent = vid;
  newVertex->m_state = sto;
  newVertex->m_type = 0; //Note: Fix this, should be 2 if in goal region.
  newVertex->m_nchildren = 0;

  MotionPlanner::AddVertex(&newVertex);
  


  
}

void MotionPlanner::ExtendRandom(void)
{
    Clock clk;
    StartTime(&clk);

//your code
    
    m_totalSolveTime += ElapsedTime(&clk);
}

void MotionPlanner::ExtendRRT(void)
{
    Clock clk;
    StartTime(&clk);
 
//your code
    
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendEST(void)
{
    Clock clk;
    StartTime(&clk);

//your code    
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

//Returns the point at the ith step on the line between config1 (a leaf of our tree) and config2 (a sample configuration), given
// a certain step size.
double[] MotionPlanner::IthStepOnLine(double[] config1, double[] config2, i){
  double x_config1 = config1[0];
  double y_config1 = config1[1];

  double x_config2 = config2[0];
  double y_config2 = config2[0];
  
  double stepSize = m_simulator->GetDistOneStep();
  double magnitude = getMagnitude(config1[0] - config2[0], config1[1] - config2[1]);

  //The x value of the point at the ith step on the line between config1 and config2 =
  // (The x val of a vector of this step size) + (displacement from origin_x to config1_x)
  //It is naturally the same for the y values.
  
  double x_iStep = ((stepSize * i)/ magnitude) * (x_config2 - x_config1) + x_config1;
  double y_iStep = ((stepSize * i)/ magnitude) * (y_config2 - y_config1) + y_config1;

  double[] iStep = {x_iStep, y_iStep};
  return iStep;
}


//Calculates the magnitude of the given vector.
double MotionPlanner::getMagnitude(double x, double y){

  return sqrt((x * x)+(y * y));
}

//Calculates magnitude of a double array vector.

double MotionPlanner::getMagnitude(double[] vector){

  return sqrt((vector[0] * vector[0]) + (vector[1] * vector[1]));

    }

