/**
 *@file Simulator.hpp
 */

#ifndef SIMULATOR_HPP_
#define SIMULATOR_HPP_

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "PseudoRandom.hpp"

class Simulator
{
public:    
    Simulator(void);
    
    ~Simulator(void);

    enum
	{
	    STATE_X = 0,
	    STATE_Y = 1,
	    STATE_NR_DIMS = 2
	};

    void SetupFromFile(const char fname[]);

    double GetRobotCenterX(void) const
    {
	return m_circles[0];	
    }

    double GetRobotCenterY(void) const
    {
	return m_circles[1];	
    }

    double GetRobotRadius(void) const
    {
	return m_circles[2];
    }
    
    double GetGoalCenterX(void) const
    {
	return m_circles[3];	
    }

    double GetGoalCenterY(void) const
    {
	return m_circles[4];	
    }

    double GetGoalRadius(void) const
    {
	return m_circles[5];
    }

    int GetNrObstacles(void) const
    {
	return m_circles.size() / 3 - 2;
    }
    
    double GetObstacleCenterX(const int i) const
    {
	return m_circles[6 + 3 * i];
    }
    
    double GetObstacleCenterY(const int i) const
    {
	return m_circles[7 + 3 * i];
    }
    
    double GetObstacleRadius(const int i) const
    {
	return m_circles[8 + 3 * i];
    }
    
    double GetDistanceFromRobotCenterToGoal(void) const
    {
	const double rx = GetRobotCenterX();	
	const double ry = GetRobotCenterY();	
	const double gx = GetGoalCenterX();
	const double gy = GetGoalCenterY();
	
	return sqrt((rx - gx) * (rx - gx) + (ry - gy) * (ry - gy));
    }

    bool HasRobotReachedGoal(void) const
    {
	return GetDistanceFromRobotCenterToGoal() <= GetGoalRadius();
    }

    void SetRobotState(const double s[])
    {
	SetRobotCenter(s[0], s[1]);
    }
    
    void SetRobotCenter(const double x, const double y)
    {
	m_circles[0] = x;
	m_circles[1] = y;
    }
    
    bool IsValidState(void) const;
  
    double GetDistOneStep(void) const
    {
	return m_distOneStep;
    }

    void SampleState(double s[]) const
    {
	s[0] = PseudoRandomUniformReal(m_bbox[0], m_bbox[2]);
	s[1] = PseudoRandomUniformReal(m_bbox[1], m_bbox[3]);
    }
    

    const double* GetBoundingBox(void) const
    {
	return m_bbox;
    }
    

protected:    
    std::vector<double> m_circles;
    double              m_distOneStep;
    double              m_bbox[4];

    friend class Graphics;
};

#endif
