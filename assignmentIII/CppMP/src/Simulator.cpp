#include "Simulator.hpp"
#include <cstring>

Simulator::Simulator(void)
{
    m_circles.push_back(-20);
    m_circles.push_back(15);
    m_circles.push_back(1.0);
    
    m_circles.push_back(20);
    m_circles.push_back(-15);
    m_circles.push_back(1.0);

    m_distOneStep = 0.2;    
}

Simulator::~Simulator(void)
{
}

void Simulator::SetupFromFile(const char fname[])
{
    FILE  *in = fopen(fname, "r");
    char   keyword[100];
    
    if(in)
    {
	while(fscanf(in, "%s", keyword) == 1)
	{
	    if(strcmp(keyword, "InitialState") == 0)
		fscanf(in, "%lf %lf %lf", &m_circles[0], &m_circles[1], &m_circles[2]);
	    else if(strcmp(keyword, "Goal") == 0)
		fscanf(in, "%lf %lf %lf", &m_circles[3], &m_circles[4], &m_circles[5]);
	    else if(strcmp(keyword, "BBox") == 0)
		fscanf(in, "%lf %lf %lf %lf", &m_bbox[0], &m_bbox[1], &m_bbox[2], &m_bbox[3]);
	    else if(strcmp(keyword, "DistOneStep") == 0)
		fscanf(in, "%lf", &m_distOneStep);
	    else if(strcmp(keyword, "Obstacles") == 0)
	    {
		int n = 0;
		double x, y, r;		
		fscanf(in, "%d", &n);
		for(int i = 0; i < n; ++i)
		{
		    fscanf(in, "%lf %lf %lf", &x, &y, &r);
		    m_circles.push_back(x);
		    m_circles.push_back(y);
		    m_circles.push_back(r);
		}
	    }
	}
	fclose(in);	    
    }	
    else
	printf("..could not open file <%s>\n", fname);
}


bool Simulator::IsValidState(void) const
{
    const double rx = GetRobotCenterX();
    const double ry = GetRobotCenterY();
    const double rr = GetRobotRadius();

    if (rx < m_bbox[0] || rx > m_bbox[2] ||
	ry < m_bbox[1] || ry > m_bbox[3])
	return false;

    const int    n  = GetNrObstacles();
    
    for(int i = 0; i < n; ++i)
    {
	const double x = GetObstacleCenterX(i);
	const double y = GetObstacleCenterY(i);
	const double r = GetObstacleRadius(i);
	const double d = sqrt((rx - x) * (rx - x) + (ry - y) * (ry - y));
	
	if(d < rr + r)
	    return false;
    }
    return true;
}
