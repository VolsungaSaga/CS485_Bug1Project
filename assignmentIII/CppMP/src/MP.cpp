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
