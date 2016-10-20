/*
 * RotationDisplay.java
 *
 * Created on October 19, 2006, 9:08 PM
 * By Sean Robinette
 */

package gmu.robot.pioneer.gui;

import java.awt.*;
import javax.swing.*;

public class RotationDisplay extends JComponent {
    double angle;
    
    public void paint(Graphics g) {
        g.setColor(Color.WHITE);
        g.fillRect(0,0,getWidth()-1, getHeight()-1);
        g.setColor(Color.BLACK);
        int x = (int)(getWidth()/2 + getWidth()/2*Math.cos(angle));
        int y = (int)(getHeight()/2 + getHeight()/2*Math.sin(angle));
        g.drawLine(getWidth()/2, getHeight()/2, x, y);
        g.drawRect(0,0,getWidth()-1, getHeight()-1);
        }
    
    public void setAngle(double newAngle) {
        angle = newAngle;
        }
    
    public RotationDisplay() {
        setSize(200,200);
        }
    
    }
