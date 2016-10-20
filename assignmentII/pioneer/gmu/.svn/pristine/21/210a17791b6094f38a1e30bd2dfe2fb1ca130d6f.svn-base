/*
 * JoystickDisplay.java
 *
 * Created on October 19, 2006, 4:10 PM
 * By Sean Robinette
 * 
 * Spruced up by Joey Harrison  
 * Last Modified 12/8/07
 */

package gmu.robot.pioneer.jgui;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;

public class JoystickDisplay extends JComponent
    {
    private static final long serialVersionUID = 1L;

    private gmu.robot.pioneer.PioneerRobot bot;
    private double xPos, yPos = 0;          // x and y position, between (-1.0 and 1.0)
    private boolean enabled = false;
        
    private int radius = 10;
    private int diameter = 20;

    public void paint(Graphics g)
        {
        int width = getWidth();
        int height = getHeight();
                
        g.setColor(Color.WHITE);
        g.fillRect(0, 0, width, height);
                
        g.setColor(enabled ? Color.green : Color.gray);
        int x,y;
                
        if (enabled)
            {
            x = width / 2 + (int) (xPos * width / 2);
            y = height / 2 + (int) (yPos * height / 2);
            } 
        else
            {
            x = width / 2;
            y = height / 2;
            }
                
        g.fillOval(x - radius, y - radius, diameter, diameter);

        //g.drawString(String.valueOf(x) + "," + String.valueOf(y), 10, 10);
                
        g.setColor(Color.BLACK);
        g.drawRect(0, 0, width - 1, height - 1);
        }

    public void setPos(double x, double y)
        {
        xPos = x;
        yPos = y;
        }

    public double getXPos()
        {
        return (xPos);
        }

    public double getYPos()
        {
        return (yPos);
        }

    public void setEnabled(boolean set)
        {
        enabled = set;
        repaint();
        }

    private void setMotors(MouseEvent e)
        {
        //TODO This function should be completely rewritten
                
        double x = (1.0 - e.getX() / (double) getWidth()) * 2 - 1;
        double y = (1.0 - e.getY() / (double) getHeight()) * 2 - 1;
        double left = (y - x) / 2;
        double right = (y + x) / 2;

        byte lbyte = (byte) (((left) * 127) / 4);
        byte rbyte = (byte) (((right) * 127) / 4);

        // System.out.println(String.format("%3d %3d", lbyte, rbyte));

        bot.vel2(lbyte, rbyte);
                
        xPos = 2 * (double) (e.getX() - getWidth() / 2) / (double) getWidth();
        yPos = 2 * (double) (e.getY() - getHeight() / 2) / (double) getHeight();
                
        //System.out.format("x: %.2f,  y: %.2f,   xPos: %.2f,  yPos: %.2f\n", x, y, xPos, yPos);
        repaint();
        }

    public JoystickDisplay(gmu.robot.pioneer.PioneerRobot robot)
        {
        bot = robot;
        addMouseMotionListener(new MouseMotionAdapter() {
            public void mouseDragged(MouseEvent e)
                {
                if (enabled)
                    setMotors(e);
                }
            });

        addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e)
                {
                if (enabled)
                    setMotors(e);
                }

            public void mouseReleased(MouseEvent e)
                {
                if (enabled)
                    {
                    bot.vel2((byte) 0, (byte) 0);
                    xPos = 0;
                    yPos = 0;
                    repaint();
                    }
                }
            });
        }

    }
