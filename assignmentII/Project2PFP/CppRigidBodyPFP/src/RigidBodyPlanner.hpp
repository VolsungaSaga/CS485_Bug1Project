#ifndef RIGID_BODY_PLANNER_HPP_
#define RIGID_BODY_PLANNER_HPP_

#include "RigidBodySimulator.hpp"

struct RigidBodyMove
{
    double m_dx;
    double m_dy;
    double m_dtheta;
};

class RigidBodyPlanner
{
public:
    RigidBodyPlanner(RigidBodySimulator * const simulator);
            
    ~RigidBodyPlanner(void);

    /*
     * This is the function that you should implement.
     * This function needs to compute by how much the position (dx, dy) 
     * and orientation (dtheta) should change so that the robot makes a small 
     * move toward the goal while avoiding obstacles, 
     * as guided by the potential field.
     *
     * You have access to the simulator.
     * You can use the methods available in simulator to get all the information
     * you need to correctly implement this function
     *
     */
    RigidBodyMove ConfigurationMove(void);

    /*!
        Multiply the point x, y, theta by the jacobian. x, y should be a control point
            in the world space
        \param[in] p_retVec - the 1x3 return vector - this holds [m_dx, m_dy, m_dtheta]
            note don't new this from the heap just create it on the stack since it is a temporary.
        \param[in] p_x - the x coordinate in the world space, x control point.
        \param[in] p_y - the y coordinate in the world space, y control point.
        \param[in] p_theta - the world space configuration angle. - ***Should be in radians!!!
        \param[in] p_scaleXY the scale factor for the x, y point.
        \param[in] p_scaleTheta - the scale factor for theta. 
     */
    void worldSpaceToConfigSpace(std::vector<double> p_retVector, double p_xControl, double p_yControl,
        double p_theta, double DUx, double DUy, double p_scaleXY, double p_scaleTheta) const
    {
        // the first two values of our vector dx and dy are simply x and y.
        p_retVector.push_back(p_scaleXY * DUx);
        p_retVector.push_back(p_scaleXY * DUy);
        
        // first part of the matrix i.e. -X_j * sin(theta) - Y_j * cos(theta) * DUx
        double accumulator = -p_xControl * std::sin(p_theta) - p_yControl * std::cos(p_theta) * DUx;

        // accumulator now has (-X_j * sin(theta) - Y_j * cos(theta) * DUx) +
        //                     (X_j * cos(theta) - Y_j * sin(theta) * DUy)
        accumulator += p_xControl * std::cos(p_theta) - p_yControl * std::sin(p_theta) * DUy;
        p_retVector.push_back(p_scaleTheta * accumulator);
         
    }
     
protected:
    RigidBodySimulator *m_simulator;

};

#endif
