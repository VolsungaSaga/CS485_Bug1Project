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


//Our code goes here
void MotionPlanner::ExtendTree(const int    vid, 
			       const double sto[])
{
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
    
    m_totalSolveTime += ElapsedTime(&clk);
}

void MotionPlanner::ExtendRRT(void)
{
    Clock clk;
    StartTime(&clk);
    //Here, we check to see how close our robot is to our goal's radius
    double goalWRadiusX = m_simulator->GetGoalCenterX()+m_simulator->GetRobotRadius();
    double goalWRadiusY = m_simulator->GetGoalCenterY()+m_simulator->GetRobotRadius();
    double sto[2]; // This is our step val;
    sto[0] = PseudoRandomUniformRead(0,1);
    sto[1] = PseudoRandomUniformRead(0,1);   

    int i = 0; // This is our check val
    while ((goalWRadiusX < m_vertices[i]) && (goalWRadiusY < m_vertices[i])){
        ExtendTree(i,sto);
        if (m_vertices[i+1] == NULL) {
           if (goalWRadiusX > 0)
               sto[0] = (m_vertices[i])+PseudoRandomUniformRead(0,1);
           else if (goalWRadiusX < 0)
               sto[0] = (m_vertices[i])-PseudoRandomUniformRead(0,1);
           if (goalWRadiusY > 0)
               sto[1] = (m_vertices[i])+PseudoRandomUniformRead(0,1);
           else if (goalWRadiusY < 0)
               sto[1] = (m_vertices[i])-PseudoRandomUniformRead(0,1);
        }
        else {
           i++;
        }
    }
    GetPathFromInitToGoal(m_vertices);
    m_totalSolveTime += ElapsedTime(&clk);
}

//q == tree configuration
//1/(1+Deg(q)) is our weight; also our probablility
//Deg(q) == numbers of neighbors near our q
void MotionPlanner::ExtendEST(void)
{
    Clock clk;
    StartTime(&clk);
    //IMPORTANT NOTE: NEEDS A LOTTA WORK
    //Here, we check to see how close our robot is to our goal's radius
    double goalWRadiusX = m_simulator->GetGoalCenterX()+m_simulator->GetRobotRadius();
    double goalWRadiusY = m_simulator->GetGoalCenterY()+m_simulator->GetRobotRadius();
    double sto[2]; // This is our step val;
    sto[0] = PseudoRandomUniformRead(0,1);
    sto[1] = PseudoRandomUniformRead(0,1);

    int i = 0; // This is our check val
    int e = 1;
    while ((goalWRadiusX < m_vertices[i]) && (goalWRadiusY < m_vertices[i])){
        sto[0] = ((1/(1+i))/(e*PseudoRandomUniformRead(0,1)));
        sto[1] = ((1/(1+i))/(e*PseudoRandomUniformRead(0,1)));
        ExtendTree(i,sto);
        if (m_vertices[i+1] != NULL) {
            i++;
        }
        e++; 
    }   
    
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendMyApproach(void)
{
    Clock clk;
    StartTime(&clk);

       // Here, what we're trying to do is that we want
    // to first make a possible path that could stem from our decisions
    double vidGo[2] = {0,0};
    double vidGoal[2] = {0,0};
    vidGo[0] = m_simulator->GetRobotCenterX();
    vidGo[1] = m_simulator->GetRobotCenterY();

    vidGoal[0] = m_simulator->GetGoalCenterX();
    vidGoal[1] = m_simulator->GetGoalCenterY();

    //Here, what we are doing is that we're setting up the line for our system
    double line[] = {vidGoal[0] - vidGo[0], vidGoal[1] - vidGo[1]};
    double magnitudeLine = this->getMagnitude(line);
    int numStepsOnLine = floor(magnitudeLine/(m_simulator->GetDistOneStep()));


    double sto[2];
    double iStepVector[2] = {0,0}; //Typically step size when no objects present
    long e = m_vertices.size(); // Checks to see what vertex we are currently at

    long randomVertex = 0;
    int j = 0;


    /*IthStepOnLine(vidGo,vidGoal,1,m_simulator->GetDistOneStep(), iStepVector);
    printf("iStepVector X:%1.2f\n",iStepVector[0]);
    printf("iStepVector Y:%1.2f\n",iStepVector[1]);
    exit(0);*/
    for (int i = 0; i < numStepsOnLine; i++) {
      for (int j= 0; j < m_simulator->GetNrObstacles(); j++) {
        e = m_vertices.size();
        if (e != 0)
          randomVertex = random() % e;
        double d = sqrt(pow((vidGo[0]-m_simulator->GetObstacleCenterX(j)),2)+pow((vidGo[1]-m_simulator->GetObstacleCenterY(j)),2));
        if (d < (m_simulator->GetObstacleRadius(j))) {
          m_simulator->SampleState(sto);
          ExtendTree(randomVertex, sto);
        }
        else {
          ExtendTree(e-1,iStepVector);
        }
      }
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
