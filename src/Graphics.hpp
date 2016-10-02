/**
 *@file Graphics.hpp
 *@author Erion Plaku 
 *@brief Graphics for running simulation and setting up problem
 */

#ifndef  GRAPHICS_HPP_
#define  GRAPHICS_HPP_

#include "BugAlgorithms.hpp"
#include "Simulator.hpp"

/**
 *@author Erion Plaku 
 *@brief  Graphics for running simulation and setting up problem
 */
class Graphics
{   
public:
   /**
    *@brief Initialize data and variables
    *
    *@param fname name of file with obstacles
    *@param bugSelection selected bug algorithm (0, 1, or 2)
    */
    Graphics(const char fname[], const int bugSelection);
    
   /**
    *@brief Destroy window
    */
    ~Graphics(void);

   /**
    *@brief Print help information
    */
    void HandleEventOnHelp(void);
 
   
   /**
    *@brief Main event loop
    */
    void MainLoop(void);

protected:
   /**
    *@brief Perform simulation step
    */
    void HandleEventOnTimer(void);
    
   /**
    *@brief Main rendering function
    */
    void HandleEventOnDisplay(void);
    
   /**
    *@brief Respond to event when left button is clicked
    *
    *@param mousePosX x-position
    *@param mousePosY y-position
    */
    void HandleEventOnMouseLeftBtnDown(const double mousePosX, const double mousePosY);
    
   /**
    *@brief Respond to key presses
    *
    *@param key key pressed
    */
    void HandleEventOnKeyPress(const int key);

   /**
    *@brief Draw circle
    *
    *@param cx x position of circle center
    *@param cy y position of circle center
    *@param r circle radius
    */
    void DrawCircle2D(const double cx, const double cy, const double r);

   /**
    *@name GLUT callback functions
    *@{
    */

    static void CallbackEventOnDisplay(void);
    static void CallbackEventOnMouse(int button, int state, int x, int y);
    static void CallbackEventOnTimer(int id);
    static void CallbackEventOnKeyPress(unsigned char key, int x, int y);
    static void MousePosition(const int x, const int y, double *posX, double *posY);

   /**
    *@}
    */
 
   /**
    *@brief An instance of the simulator
    */
    Simulator m_simulator;
    
   /**
    *@brief Pointer to an instance of the bug algorithms
    */
    BugAlgorithms *m_bugAlgorithms;
    
   /**
    *@brief Ensure that user sets up robot and goal centers
    */
    int m_setRobotAndGoalCenters;

    /**
     *@brief Bug algorithm selected by user
     */
    int m_bugSelection;

  void ExportFrameAsImage(void);
    void ExportFrameAsImage(const char fname[]);
 
        int  m_frames;
    bool m_exportFrames;

    
};

#endif
