#include "Graphics.hpp"

#ifdef __APPLE__
#include <GLUT/glut.h>
#elif defined _WIN32
#include "glutForWindows.h"
#else
#include <GL/glut.h>
#endif

Graphics *m_graphics = NULL;

Graphics::Graphics(const char fname[]) 
{
    m_simulator.SetupFromFile(fname);
    m_planner = new MotionPlanner(&m_simulator);

    m_selectedCircle = -1;
    m_editRadius     = false;
    m_run            = false;

    m_pathPos = 0;

    m_drawPlannerVertices = true;

    m_method = 2;    
}

Graphics::~Graphics(void)
{
    if(m_planner)
	delete m_planner;
}

void Graphics::MainLoop(void)
{	
    m_graphics = this;

//create window    
    static int    argc = 1;	
    static char  *args = (char*)"args";
    glutInit(&argc, &args);    
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);    
    glutInitWindowSize(1000, 600);
    glutInitWindowPosition(0, 0); 
    glutCreateWindow("Planner");	   	


//register callback functions
    glutDisplayFunc(CallbackEventOnDisplay);
    glutMouseFunc(CallbackEventOnMouse);
    glutMotionFunc(CallbackEventOnMouseMotion);
    glutIdleFunc(NULL);
    glutTimerFunc(1, CallbackEventOnTimer, 0); 
    glutKeyboardFunc(CallbackEventOnKeyPress);

//enter main event loop
    glutMainLoop();	
}

void Graphics::HandleEventOnTimer(void)
{
    if(m_run && m_method >= 1 && m_method <= 4)
    {
	for(int i = 0; i < 1000 && !m_planner->IsProblemSolved(); ++i)
	{
	    if(m_method == 1)      m_planner->ExtendRandom();
	    else if(m_method == 2) m_planner->ExtendRRT();
	    else if(m_method == 3) m_planner->ExtendEST();
	    else if(m_method == 4) m_planner->ExtendMyApproach();
	}
	if(!m_planner->IsProblemSolved())
	    printf("TotalSolveTime = %f [Solved = %d] [NrVertices = %d]\n", 
		   m_planner->m_totalSolveTime, m_planner->IsProblemSolved(), m_planner->m_vertices.size());
    }
     
    if(m_path.size() == 0 && m_planner->IsProblemSolved())
    {
	m_pathPos = 0;
	m_planner->GetPathFromInitToGoal(&m_path);
	
	printf("TotalSolveTime = %f [Solved = %d] [NrVertices = %d]\n", 
	       m_planner->m_totalSolveTime, m_planner->IsProblemSolved(), m_planner->m_vertices.size());
	
    }
    
    if(m_path.size() != 0)
    {
	if(m_pathPos >= m_path.size())
	    m_pathPos = 0;
	
	m_simulator.SetRobotState(m_planner->m_vertices[m_path[m_pathPos]]->m_state);
	++m_pathPos;
    }
} 

void Graphics::HandleEventOnMouseMotion(const double mousePosX, const double mousePosY)
{
    if(m_planner->m_vertices.size() > 1)
	return;
    

    if(m_selectedCircle >= 0)
    {
	if(m_editRadius)
	{
	    const double cx = m_simulator.m_circles[3 * m_selectedCircle];
	    const double cy = m_simulator.m_circles[3 * m_selectedCircle + 1];
	    
	    m_simulator.m_circles[3 * m_selectedCircle + 2] = sqrt((cx - mousePosX) * (cx - mousePosX) +
								   (cy - mousePosY) * (cy - mousePosY));
	}
	else
	{
	    m_simulator.m_circles[3 * m_selectedCircle] = mousePosX;
	    m_simulator.m_circles[3 * m_selectedCircle + 1] = mousePosY;
	}
	
    }
    
}

void Graphics::HandleEventOnMouseBtnDown(const int whichBtn, const double mousePosX, const double mousePosY)
{   
    if(m_planner->m_vertices.size() > 1)
	return;
 
    m_selectedCircle = -1;
    for(int i = 0; i < m_simulator.m_circles.size() && m_selectedCircle == -1; i += 3)
    {
	const double cx = m_simulator.m_circles[i];
	const double cy = m_simulator.m_circles[i + 1];
	const double r  = m_simulator.m_circles[i + 2];
	const double d  = sqrt((mousePosX - cx) * (mousePosX - cx) + (mousePosY - cy) * (mousePosY - cy));
	
	if(d <= r)
	    m_selectedCircle = i / 3;
    }
    
    if(m_selectedCircle == -1)
    {
	m_simulator.m_circles.push_back(mousePosX);
	m_simulator.m_circles.push_back(mousePosY);
	m_simulator.m_circles.push_back(1.0);
    }    
}

void Graphics::HandleEventOnKeyPress(const int key)
{
    FILE *out;
    
    switch(key)
    {
    case 27: //escape key
	exit(0);
	
    case 'r':
	m_editRadius = !m_editRadius;	
	break;
	
    case 'p':
	m_run = !m_run;
	printf("ALLOW RUNNING = %d\n", m_run);
	break;

    case 'v':
	m_drawPlannerVertices = !m_drawPlannerVertices;
	break;

    case '1': case '2': case '3': case '4':
	m_run    = true;	
	m_method = key - '0';
	break;
	
    case 's':
	out = fopen("obst.txt", "w");
	fprintf(out, "%d\n", m_simulator.GetNrObstacles());	
	for(int i = 0; i < m_simulator.GetNrObstacles(); ++i)
	    fprintf(out, "%f %f %f\n", 
		    m_simulator.GetObstacleCenterX(i),
		    m_simulator.GetObstacleCenterY(i),
		    m_simulator.GetObstacleRadius(i));
	fclose(out);
    }
}

void Graphics::HandleEventOnDisplay(void)
{
//draw bounding box
    const double *bbox = m_graphics->m_simulator.m_bbox;
    
    glColor3f(0, 0, 1);    
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_POLYGON);
    glVertex2d(bbox[0], bbox[1]);
    glVertex2d(bbox[2], bbox[1]);
    glVertex2d(bbox[2], bbox[3]);
    glVertex2d(bbox[0], bbox[3]);
    glEnd();
    
//draw robot, goal, and obstacles
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glColor3f(1, 0, 0);
    DrawCircle2D(m_simulator.GetRobotCenterX(), m_simulator.GetRobotCenterY(), m_simulator.GetRobotRadius());
    glColor3f(0, 1, 0);
    DrawCircle2D(m_simulator.GetGoalCenterX(), m_simulator.GetGoalCenterY(), m_simulator.GetGoalRadius());
    glColor3f(0, 0, 1);
    for(int i = 0; i < m_simulator.GetNrObstacles(); ++i)
	DrawCircle2D(m_simulator.GetObstacleCenterX(i),
		     m_simulator.GetObstacleCenterY(i),
		     m_simulator.GetObstacleRadius(i));

//draw planner vertices
    if(m_drawPlannerVertices)
    {
	glPointSize(4.0);
	
	const int n = m_planner->m_vertices.size();
	glColor3f(0.6, 0.8, 0.3);	
	glBegin(GL_POINTS);	
	for(int i = 0; i < n; ++i)
	    glVertex2dv(m_planner->m_vertices[i]->m_state);
	glEnd();
	glBegin(GL_LINES);	
	for(int i = 1; i < n; ++i)
	{
	    glVertex2dv(m_planner->m_vertices[i]->m_state);
	    glVertex2dv(m_planner->m_vertices[m_planner->m_vertices[i]->m_parent]->m_state);
	}
	glEnd();
    }
}


void Graphics::DrawCircle2D(const double cx, const double cy, const double r)
{
    const int    nsides = 50;    
    const double angle  = 2 * M_PI / nsides;
    
    glBegin(GL_POLYGON);
    for(int i = 0; i <= nsides; i++)
	glVertex2d(cx + r * cos(i * angle), cy + r * sin(i * angle));
    glEnd();	
}


void Graphics::CallbackEventOnDisplay(void)
{
    if(m_graphics)
    {
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);	
	
	glViewport(0, 0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	const double *bbox = m_graphics->m_simulator.m_bbox;
	

	glOrtho(bbox[0] - 1, bbox[2] + 1, bbox[1] - 1, bbox[3] + 1, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();	    
	
	m_graphics->HandleEventOnDisplay();
	
	glutSwapBuffers();	    
    }
}

void Graphics::CallbackEventOnMouse(int button, int state, int x, int y)
{
    if(m_graphics &&  state == GLUT_DOWN)
    {
	double mouseX, mouseY;
	MousePosition(x, y, &mouseX, &mouseY);
	m_graphics->HandleEventOnMouseBtnDown(button, mouseX , mouseY);
	glutPostRedisplay();
    }	    
}

void Graphics::CallbackEventOnMouseMotion(int x, int y)
{
    double mouseX, mouseY;
    MousePosition(x, y, &mouseX, &mouseY);
    m_graphics->HandleEventOnMouseMotion(mouseX , mouseY);
    glutPostRedisplay();
}


void Graphics::CallbackEventOnTimer(int id)
{
    if(m_graphics)
    {
	m_graphics->HandleEventOnTimer();
	glutTimerFunc(15, CallbackEventOnTimer, id);
	glutPostRedisplay();	    
    }
}



void Graphics::CallbackEventOnKeyPress(unsigned char key, int x, int y)
{
    if(m_graphics)
	m_graphics->HandleEventOnKeyPress(key);	
}


void Graphics::MousePosition(const int x, const int y, double *posX, double *posY)
{
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posZ;
    
    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );
    
    winX = (float)x;
    winY = (float)viewport[3] - (float)y;
    glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
    
    gluUnProject(winX, winY, winZ, modelview, projection, viewport, posX, posY, &posZ);
}

int main(int argc, char **argv)
{
    PseudoRandomSeed();
    
    if(argc < 2)
    {
	printf("missing arguments\n");		
	printf("  Planner <file>\n");
	return 0;		
    }

    Graphics graphics(argv[1]);
    
    graphics.MainLoop();
    
    return 0;    
}
