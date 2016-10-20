/*
 * Bar.java
 *
 * Created on October 19, 2006, 8:36 PM
 * By Sean Robinette
 */

package gmu.robot.pioneer.gui;

import java.awt.*;
import javax.swing.*;

public class Bar extends JComponent {
    double value = 0.5;
    Color bgColor;
    Color barColor;
    int numTicks = 1;
    
    public void paint(Graphics g) {
        g.setColor(bgColor);
        g.fillRect(0,0,getWidth()-1,getHeight()-1);
        g.setColor(barColor);
        g.fillRect(1,1,(int)((getWidth()-1)*value),getHeight()-1);
        g.setColor(Color.BLACK);
        g.drawRect(0,0,getWidth()-1, getHeight()-1);
        g.setColor(Color.WHITE);
        for(int i=0;i<numTicks;i++) {
            int x = getWidth()*(i+1)/(i+2);
            g.drawLine(x, 0, x, getHeight()-1);
            }
        }
    
    public void setValue(double newVal) {
        value = newVal;
        repaint();
        }
    
    public Bar() {
        bgColor = Color.decode("#333333");
        barColor = Color.GREEN;
        }
    
    }
