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
};

#endif
