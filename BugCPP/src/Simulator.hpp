/**
 *@file Simulator.hpp
 *@brief Simulate robot motion and sensor
 */

#ifndef SIMULATOR_HPP_
#define SIMULATOR_HPP_

//#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

/**
 *@brief Sensor reading
 *@param m_dmin : minimum distance from robot center to obstacle boundary
 *@param m_xmin : x-coordinate of obstacle boundary point that is closest to robot center
 *@param m_ymin : y-coordinate of obstacle boundary point that is closest to robot center
 */
struct Sensor
{
    double m_xmin;
    double m_ymin;
    double m_dmin;
};
    

/**
 *@brief Simulate robot motion and sensor
 *
 *@remark
 *  ::Simulator provides functionality to simulate the robot motion and the sensor.
 *    Simulator is considered as an input to your implementation of
 *    the bug algorithm. As such, you should not make changes to ::Simulator.
 *    In particular, your bug algorithm should only access the public functions
 *    of ::Simulator. 
 */      
class Simulator
{
public:    
    /**
     *@brief Initialize variables
     */
    Simulator(void);
    
    /**
     *@brief Delete allocated memory
     */
    ~Simulator(void);

    /**
     *@brief Take sensor reading from robot center
     */
    Sensor TakeSensorReading(void) const;

    /**
     *@brief Determine if two points (x1, y1) and (x2, y2) are near each other
     */
    bool ArePointsNear(const double x1, const double y1, 
		       const double x2, const double y2) const
    {
	return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) < 0.2;	
    }

    /**
     *@brief Determine if point (px, py) is near the line defined
     *       by its two endpoints (s1x, s1y) and (s2x, s2y)
     */
    bool IsPointNearLine(const double px,  const double py,
			 const double s1x, const double s1y,
			 const double s2x, const double s2y) const
    {
	const double x  = s2x - s1x;
	const double y  = s2y - s1y;
	const double d  = (px - s1x) * y - (py - s1y) * x;

	return d * d / (x * x + y * y) < 0.064;
    }

    /**
     *@brief Return value to determine when  to follow obstacle boundary
     */
    double GetWhenToTurn(void) const
    {
	return 1.0;
    }
      
    /**
     *@brief Step length that the robot can take
     *@remark
     * Use this value when determining the dx and dy robot displacements.
     * You need to ensure that the robot is taking small steps, i.e.,
     * displacement (sqrt(dx * dx + dy * dy))  
     * should be equal to step
     */
    double GetStep(void) const
    {
	return 0.06;
    }
               
    /**
     *@brief Get x-coordinate of robot's center
     */
    double GetRobotCenterX(void) const
    {
	return m_robotCenterX;	
    }

    /**
     *@brief Get y-coordinate of robot's center
     */
    double GetRobotCenterY(void) const
    {
	return m_robotCenterY;	
    }

    /**
     *@brief Get x-coordinate of robot's initial position
     */
    double GetRobotInitX(void) const
    {
	return m_robotInitX;	
    }

    /**
     *@brief Get y-coordinate of robot's initial position
     */
    double GetRobotInitY(void) const
    {
	return m_robotInitY;	
    }
 
    /**
     *@brief Get x-coordinate of goal center
     */
    double GetGoalCenterX(void) const
    {
	return m_goalCenterX;	
    }

    /**
     *@brief Get y-coordinate of goal center
     */
    double GetGoalCenterY(void) const
    {
	return m_goalCenterY;	
    }

    /**
     *@brief Get distance from the robot center to the goal
     */
    double GetDistanceFromRobotToGoal(void) const
    {
	return
	    sqrt((m_robotCenterX - m_goalCenterX) * (m_robotCenterX - m_goalCenterX) +
		 (m_robotCenterY - m_goalCenterY) * (m_robotCenterY - m_goalCenterY));	
    }

    /**
     *@brief Returns true iff the robot center is inside the goal circle
     */
    bool HasRobotReachedGoal(void) const
    {
	return
	    ArePointsNear(m_robotCenterX, m_robotCenterY,
			  m_goalCenterX,  m_goalCenterY);
    }


protected:
    /**
     *@brief Set robot center
     *
     *@param cx x position of center
     *@param cy y position of center
     */
    void SetRobotCenter(const double cx, const double cy)
    {
	m_robotCenterX = cx;
	m_robotCenterY = cy;
	
	m_path.push_back(cx);
	m_path.push_back(cy);
    }

    /**
     *@brief Set goal center
     *@param x x position of the goal center
     *@param y y position of the goal center
     */ 
    void SetGoalCenter(const double x, const double y)
    {
	m_goalCenterX = x;
	m_goalCenterY = y;	
    }

    /**
     *@brief Read polygonal obstacles from input file
     *
     *@param fname name of obstacle file
     */ 	
    void ReadObstacles(const char fname[]);


    /**
     *@brief Robot's current position
     */
    double m_robotCenterX;
    double m_robotCenterY;

    /**
     *@brief Robot's initial position
     */
    double m_robotInitX;
    double m_robotInitY;
    

    /**
     *@brief Goal's position
     */
    double m_goalCenterX;
    double m_goalCenterY;

    /**
     *@brief Geometry of the obstacles
     *@remark
     *  Each obstacle corresponds to a polygon
     */
    struct Obstacle
    {
	std::vector<double> m_vertices;
	std::vector<int>    m_triangles;
    };
	
    std::vector<Obstacle *> m_obstacles;
    std::vector<double>     m_obstacleEdges;
        
    /**
     *@brief For storing the history of robot positions
     */
    std::vector<double> m_path;

    friend class Graphics;
};

#endif
