/*
 * JoystickDisplay.java
 *
 * Created on October 19, 2006, 4:10 PM
 * By Sean Robinette
 */

package gmu.robot.pioneer.gui;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import gmu.robot.pioneer.*;

public class JoystickDisplay extends JComponent {
    
    private gmu.robot.pioneer.PioneerRobot bot;
    private double xPos,yPos = 0;
    private boolean enabled = false;
    private boolean mouse = false;
    
    public void paint(Graphics g) {
        g.setColor(Color.WHITE);
        g.fillRect(0,0,getWidth(),getHeight());
        if(enabled) {
            if(mouse) {
                g.setColor(Color.BLUE);
                } else {
                g.setColor(Color.GREEN);
                }
            int x = getWidth()/2 + (int)(xPos*getWidth()/2);
            int y = getHeight()/2 + (int)(yPos*getHeight()/2);
            g.fillOval(x-getWidth()/20, y-getWidth()/20, getWidth()/10, getWidth()/10);
            g.drawString(String.valueOf(x) + "," + String.valueOf(y), 10, 10);
            } else {
            g.setColor(Color.RED);
            g.fillOval(getWidth()/2 - getWidth()/20, getHeight()/2 - getHeight()/20, getWidth()/10, getHeight()/10);
            }
        g.setColor(Color.BLACK);
        g.drawRect(0,0,getWidth()-1,getHeight()-1);
        }
    
    public void setPos(double x, double y) {
        xPos = x;
        yPos = y;
        }
    public double getXPos() {
        return(xPos);
        }
    public double getYPos() {
        return(yPos);
        }
    public void setEnabled(boolean set) {
        enabled = set;
        }
    public void setMouse(boolean set) {
        mouse = set;
        }
    void setMotors(MouseEvent e) {
        double x = (1.0 - e.getX() / (double)getWidth()) * 2 - 1;
        double y = (1.0 - e.getY() / (double)getHeight()) * 2 - 1;
        double left = (y - x) / 2;
        double right = (y + x) / 2;
        bot.vel2((byte)(((left) * 127) / 4), (byte)(((right) *127) / 4));
        xPos = 2*(double)(e.getX() - getWidth()/2) / (double)getWidth();
        yPos = 2*(double)(e.getY() - getHeight()/2) / (double)getHeight();
        repaint();
        }
    
    public JoystickDisplay(gmu.robot.pioneer.PioneerRobot robot) {
        bot = robot;
        addMouseMotionListener(new MouseMotionAdapter()
            {
            public void mouseDragged(MouseEvent e)
                {
                if(mouse)
                    setMotors(e);
                }
            });

        addMouseListener(new MouseAdapter()
            {
            public void mousePressed(MouseEvent e)
                {
                if(mouse)
                    setMotors(e);
                }

            public void mouseReleased(MouseEvent e)
                {
                if(mouse) {
                    bot.vel2((byte)0,(byte)0);
                    xPos = 0;
                    yPos = 0;
                    repaint();
                    }
                }
            });
        }

    
    }
