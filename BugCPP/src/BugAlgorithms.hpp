/**
 *@file BugAlgorithms.hpp
 *@brief Prototype for the Bug algorithms required in this assignment
 */

#ifndef BUG_ALGORITHMS_HPP_
#define BUG_ALGORITHMS_HPP_

#include "Simulator.hpp"

/**
 * @brief Bug algorithm computes a small move (m_dx, m_dy) that the robot needs to make
 */
struct Move
{
    double m_dx;
    double m_dy;    
};


/**
 *@brief Prototype for the different Bug algorithms required in this assignment
 *
 *@remark
 *  Feel free to add additional functions/data members as you see fit
 */
class BugAlgorithms
{
public:
    /**
     *@brief Set simulator
     *@param simulator pointer to simulator
     *
     *@remark
     *  Constructor performs only pointer assignment. Therefore,
     *  destructor should not delete the simulator
     */
    BugAlgorithms(Simulator * const simulator);
            
    /**
     *@brief Free memory and delete objects allocated by this instance
     *
     *@remark
     *  Constructor performs only pointer assignment. Therefore,
     *  destructor should not delete the simulator.
     */
    ~BugAlgorithms(void);
     
    
    /**
     *@brief Select the appropriate move so that the robot behaves
     *       as described in the respective bug algorithms.
     *@param sensor provides closest point from obstacle boundary to robot center
     */
    Move Bug0(Sensor sensor);
    Move Bug1(Sensor sensor);
    Move Bug2(Sensor sensor);

  //Custom functions, mostly having to do with vector arithmetic. More
  // documentation available in the BugAlgorithms.cpp file, over each
  // particular function.

    /*!
        Checks to see if dmin is violated by checking if 
            dmin is less than when to turn
        \param[in] sensor - the sensor object
     */ 
    const bool amITooClose(const Sensor& sensor) const;
  
    /*!
        Finds a vector perpendicular to the current vector

        \param[in] x - a value representing the the difference 
            between the head of the vector (the point on the obsticle)
            and the current x position of the robot.

        \param[in] y - a value representing the the difference 
            between the head of the vector (the point on the obsticle)
            and the current y position of the robot.

        \return a move object representing the next move
     */
    Move getPerpendVector(const double x, const double y) const;

    /*!
        Finds the magnitude of the given vector

        \param[in] x - the x coordinate of the vector

        \param[in] y - the y coordinate of the vector

        \return double representing the length of the vector   
     */
    const double getMagnitude(const double x, const double y) const;

    /*!
        Find the next step of the robot using the magnitude of 
            step size from the simulator object and the vector
            
        \param[in] x - the x coordinate to find the step vector of

        \param[in] y - the y coordinate to find the step vector of.

        \return a Move object representing the next move of approprate step size. 
     */  
    Move getStepVector(const double x, const double y) const;

    /*!
        Checks to see if the robot is on the M vector
 
        \param[in] x - the current robot x
        \param[in] y - the current robot y

        \return a bool true when  near the M vector false otherwise.
     */  
    const bool onMVector(const double x, const double y) const;

    /*!
        Gets a vector that will follow the obsticle with magnitude stepsize.

        \return Move - a vector representing the next move on the proper vector.
     */
    Move follow(const Sensor& sensor, double xCurr, double yCurr) const;

protected:
    /**
     *@brief Pointer to simulator
     */
    Simulator  *m_simulator;

    enum Mode
	{
	    STRAIGHT,
	    STRAIGHT_AND_AWAY_FROM_LEAVE_POINT,
	    AROUND_AND_AWAY_FROM_HIT_POINT,
	    AROUND_AND_TOWARD_LEAVE_POINT,
	    AROUND
	};

    double m_hit[2], m_leave[2], m_distLeaveToGoal;
    int    m_mode;
    

    friend class Graphics;

private:

    //! the vector containing the x and y coordinates of the M vector.
    //! Such that m_MVector[0] is the X coordinate and m_MVector[1] is Y
    bool m_onM;
};

#endif
