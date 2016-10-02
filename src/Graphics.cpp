#include "Graphics.hpp"

#ifdef __APPLE__
#include <GLUT/glut.h>
#elif defined _WIN32
#include "glutForWindows.h"
#else
#include <GL/glut.h>
#endif

Graphics *m_graphics = NULL;

Graphics::Graphics(const char fname[], const int bugSelection) 
{
    m_simulator.ReadObstacles(fname);
    m_bugAlgorithms = new BugAlgorithms(&m_simulator);

    m_setRobotAndGoalCenters = 0;    
    m_bugSelection           = bugSelection;
        m_frames = 0;
    m_exportFrames = 0;

}

Graphics::~Graphics(void)
{
    if(m_bugAlgorithms)
	delete m_bugAlgorithms;
}

void Graphics::MainLoop(void)
{	
    m_graphics = this;

//create window    
    static int    argc = 1;	
    static char  *args = (char*)"args";
    glutInit(&argc, &args);    
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);    
    glutInitWindowSize(600, 400);
    glutInitWindowPosition(0, 0); 
    glutCreateWindow("Bug Algorithms");	   	


//register callback functions
    glutDisplayFunc(CallbackEventOnDisplay);
    glutMouseFunc(CallbackEventOnMouse);
    glutIdleFunc(NULL);
    glutTimerFunc(0, CallbackEventOnTimer, 0); 
    glutKeyboardFunc(CallbackEventOnKeyPress);

//enter main event loop
    glutMainLoop();	
}

void Graphics::HandleEventOnTimer(void)
{	
    if(m_setRobotAndGoalCenters == 2)
    {
	Move   move;    
	Sensor sensor = m_simulator.TakeSensorReading();
	
	if(!m_simulator.HasRobotReachedGoal())
	{
	    if(m_bugSelection == 0)
		move = m_bugAlgorithms->Bug0(sensor);
	    else if(m_bugSelection == 1)
		move = m_bugAlgorithms->Bug1(sensor);
	    else if(m_bugSelection == 2)
		move = m_bugAlgorithms->Bug2(sensor);

	    m_simulator.SetRobotCenter(m_simulator.GetRobotCenterX() + move.m_dx, 
				       m_simulator.GetRobotCenterY() + move.m_dy);
	    
	    if(m_exportFrames)
		ExportFrameAsImage();

	}
    }
    
} 

void Graphics::HandleEventOnMouseLeftBtnDown(const double mousePosX, const double mousePosY)
{
    if(m_setRobotAndGoalCenters == 0)
    {
	m_simulator.SetRobotCenter(mousePosX, mousePosY);
	m_simulator.m_robotInitX = mousePosX;
	m_simulator.m_robotInitY = mousePosY;	
	m_setRobotAndGoalCenters = 1;	
    }
    else if(m_setRobotAndGoalCenters == 1)
    {
	m_simulator.SetGoalCenter(mousePosX, mousePosY);
	m_setRobotAndGoalCenters = 2;	
    }
}

void Graphics::HandleEventOnKeyPress(const int key)
{
    printf("pressed key = %d\n", key);
    
    switch(key)
    {
    case 27: //escape key
	exit(0);
	
    case GLUT_KEY_F1: 
	HandleEventOnHelp();	
	break;
    case 'f':
	m_exportFrames = !m_exportFrames;
	break;	

    }
}

void Graphics::HandleEventOnHelp(void)
{
    printf("Help: left-click to desired place to set up the value for robot center and goal center\n");
}


void Graphics::HandleEventOnDisplay(void)
{
//draw robot
    if(m_setRobotAndGoalCenters >= 1)
    {
	glColor3f(1, 0, 0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	
	DrawCircle2D(m_simulator.GetRobotCenterX(), m_simulator.GetRobotCenterY(), 0.1);
    }

    
    if(m_setRobotAndGoalCenters >= 2)
    {
//draw goal
	glColor3f(0, 1, 0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	
	DrawCircle2D(m_simulator.GetGoalCenterX(), m_simulator.GetGoalCenterY(), 0.6);
    }
    
//draw trajectory    
    glColor3f(0, 0, 1);
    glLineWidth(3.0);
    glBegin(GL_LINE_STRIP);
    for(int i = 0; i < (int) m_simulator.m_path.size(); i += 2)
	glVertex2dv(&m_simulator.m_path[i]);
    glEnd();	
    glLineWidth(1.0);	

//draw obstacles
    glColor3f(0.45, 0.34, 0.76);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	
    glBegin(GL_TRIANGLES);
    for(int i = 0; i < (int) m_simulator.m_obstacles.size(); ++i)
    {
	Simulator::Obstacle *obst = m_simulator.m_obstacles[i];
	const int            ntri = obst->m_triangles.size();
    
	for(int j = 0; j < ntri; j += 3)
	{
	    glVertex2dv(&obst->m_vertices[2 * obst->m_triangles[j + 0]]);
	    glVertex2dv(&obst->m_vertices[2 * obst->m_triangles[j + 1]]);
	    glVertex2dv(&obst->m_vertices[2 * obst->m_triangles[j + 2]]);
	}
    }
    glEnd();

//draw leave point
    if(m_bugAlgorithms->m_distLeaveToGoal != HUGE_VAL)
    {
	glColor3f(0, 0, 0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	
	DrawCircle2D(m_bugAlgorithms->m_leave[0], m_bugAlgorithms->m_leave[1], 0.5);
    }

    if(m_bugAlgorithms->m_hit[0] != HUGE_VAL)
    {
	glColor3f(0, 0.7, 0.7);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	
	DrawCircle2D(m_bugAlgorithms->m_hit[0], m_bugAlgorithms->m_hit[1], 0.5);
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
	glOrtho(-22, 22, -14, 14, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();	    
	
	m_graphics->HandleEventOnDisplay();
	
	glutSwapBuffers();	    
    }
}

void Graphics::CallbackEventOnMouse(int button, int state, int x, int y)
{
    if(m_graphics && button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
	double mouseX, mouseY;
	MousePosition(x, y, &mouseX, &mouseY);
	m_graphics->HandleEventOnMouseLeftBtnDown(mouseX , mouseY);
	glutPostRedisplay();
    }	    
}

void Graphics::CallbackEventOnTimer(int id)
{
    if(m_graphics)
    {
	m_graphics->HandleEventOnTimer();
	glutTimerFunc(1, CallbackEventOnTimer, id);
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

void Graphics::ExportFrameAsImage(void)
{
    char fname[50];
    sprintf(fname, "frames_%05d.ppm", m_frames++);		    
    ExportFrameAsImage(fname);
}

void Graphics::ExportFrameAsImage(const char fname[])
{
    
    const int width = glutGet(GLUT_WINDOW_WIDTH);
    const int height= glutGet(GLUT_WINDOW_HEIGHT);
    
    char *temp  = new char[3 * width * height];
    char *image = new char[3 * width * height];
    
    FILE *fp = fopen(fname, "w");
    
    printf("Writing %s\n", fname);
    
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, temp);
    
    int  a, b, row_sz = 3*width;
    // Reverse rows
    for(int i=0; i < height; i+=1) 
    {
	for(int j=0; j < width; j+=1) 
	{
	    a = i*row_sz+3*j;
	    b = (height-i-1)*row_sz+3*j;
	    image[a]   = temp[b];
	    image[a+1] = temp[b+1];
	    image[a+2] = temp[b+2];
	}
    }
    fprintf(fp, "P6\n");
    fprintf(fp, "%i %i\n 255\n", width, height);
    fwrite(image, sizeof(char), 3 * width * height, fp);
    fclose(fp);	    
    delete[] temp;
    delete[] image;
}

int main(int argc, char **argv)
{
    if(argc < 3)
    {
	printf("missing obstacle file argument\n");		
	printf("  RunBug <obstacle_file.txt> <bugSelection>\n");
	return 0;		
    }

    Graphics graphics(argv[1], atoi(argv[2]));
    
    graphics.HandleEventOnHelp();
    graphics.MainLoop();
    
    return 0;    
}
