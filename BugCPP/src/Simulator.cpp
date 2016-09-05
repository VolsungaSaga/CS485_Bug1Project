#include "Simulator.hpp"

Simulator::Simulator(void)
{
    m_robotCenterX = m_robotInitX = -20;
    m_robotCenterY = m_robotInitY =  13;
    
    m_goalCenterX  = 16;
    m_goalCenterY  = -12;    
}

Simulator::~Simulator(void)
{
    const int n = m_obstacles.size();
    for(int i = 0; i < n; ++i)
	if(m_obstacles[i])
	    delete m_obstacles[i];    
}

double PointSegmentDistanceSquare(const double p[],
				  const double s0[],
				  const double s1[],
				  double pmin[]);


double PointPolygonDistanceSquare(const double p[],
				  const int    n,
				  const double poly[],
				  double pmin[]);

Sensor Simulator::TakeSensorReading(void) const
{
    const double  c[2] = {m_robotCenterX, m_robotCenterY};
    double        d    = 0;
    double        pmin[2];
    Sensor        sensor;
    
    sensor.m_dmin = HUGE_VAL;
    
    for(int i = 0; i < (int) m_obstacles.size(); ++i)
    {
	const int     nv   = m_obstacles[i]->m_vertices.size() / 2;
	const double *poly = &(m_obstacles[i]->m_vertices[0]);
	
	if((d = PointPolygonDistanceSquare(c, nv, poly, pmin)) < sensor.m_dmin)
	{
	    sensor.m_dmin = d;
	    sensor.m_xmin = pmin[0];
	    sensor.m_ymin = pmin[1];
	}   
    }

    return sensor;
}


void Simulator::ReadObstacles(const char fname[])
{
    FILE *in = fopen(fname, "r");
    if(in)
    {
	int       n;	
	Obstacle *obst;
	
	while(fscanf(in, "%d", &n) == 1)
	{
	    obst = new Obstacle();
	    obst->m_vertices.resize(2 * n);	    
	    for(int i = 0; i < 2 * n; ++i)
	 	fscanf(in, "%lf", &(obst->m_vertices[i]));
	    obst->m_triangles.resize(3 * (n - 2));
	    for(int i = 0; i < (int) obst->m_triangles.size(); ++i)
		fscanf(in, "%d", &(obst->m_triangles[i]));
	    m_obstacles.push_back(obst);	    

	    for(int i = 0; i < n - 1; ++i)
	    {
		m_obstacleEdges.push_back(obst->m_vertices[2 * i    ]);
		m_obstacleEdges.push_back(obst->m_vertices[2 * i + 1]);
		m_obstacleEdges.push_back(obst->m_vertices[2 * i + 2]);
		m_obstacleEdges.push_back(obst->m_vertices[2 * i + 3]);
	    }
	    m_obstacleEdges.push_back(obst->m_vertices[2 * n - 2]);
	    m_obstacleEdges.push_back(obst->m_vertices[2 * n - 1]);
	    m_obstacleEdges.push_back(obst->m_vertices[0]);
	    m_obstacleEdges.push_back(obst->m_vertices[1]);
	}
	fclose(in);	    
    }	
    else
	printf("..could not open file <%s>\n", fname);
    
}


double PointSegmentDistanceSquare(const double p[],
				  const double s0[],
				  const double s1[],
				  double pmin[])
{
    double a, b;
    double v[2] = {s1[0] - s0[0], s1[1] - s0[1]};
    double w[2] = {p [0] - s0[0], p [1] - s0[1]};
    
    if((a = (v[0] * w[0] + v[1] * w[1])) <= 0)
    {
	pmin[0] = s0[0];
	pmin[1] = s0[1];
	return w[0] * w[0] + w[1] * w[1];
    }	
    if((b = (v[0] * v[0] + v[1] * v[1])) <= a)
    {
	pmin[0] = s1[0];
	pmin[1] = s1[1];
	
	return (p[0] - s1[0]) * (p[0] - s1[0]) +
	       (p[1] - s1[1]) * (p[1] - s1[1]);
    }
    a   /= b;
    v[0] = s0[0] + a * v[0];
    v[1] = s0[1] + a * v[1];
    
    pmin[0] = v[0];
    pmin[1] = v[1];
    
    return (p[0] - v[0]) * (p[0] - v[0]) +  (p[1] - v[1]) * (p[1] - v[1]);
}

double PointPolygonDistanceSquare(const double p[],
				  const int    n,
				  const double poly[],
				  double pmin[])
{
    double dmin = PointSegmentDistanceSquare(p, &poly[2 * n - 2], &(poly[0]), pmin);
    double d;
    double lpmin[2];
    int    i;
    
    for(i = 0; i < n - 1; ++i)
	if((d = PointSegmentDistanceSquare(p, &poly[2 * i], &poly[2 * i + 2], lpmin)) < dmin)
	{
	    pmin[0] = lpmin[0];
	    pmin[1] = lpmin[1];
	    dmin = d;
	}
    return dmin;
}
