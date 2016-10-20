/*
 * SonarDisplay.java
 *
 * Created on October 11, 2006, 6:13 PM
 * by Sean Robinette
 */

package gmu.robot.pioneer.gui;

import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import gmu.robot.pioneer.*;

public class SonarDisplay extends JComponent {
        
    public volatile double[] sonars;    // in mm
    public double[] angles;             // in radians
    public int[] x,y;                   // in mm
    public int width,height;
    public double spreadAngle;
    public double scaleFactor;
    private boolean enabled = false;
    
    public SonarDisplay(double[] newSonars) {
        setSize(200,200);
        width = getWidth();
        height = getHeight();
        
        sonars = newSonars;
        angles = new double[16];
        x = new int[16];
        y = new int[16];
        
        scaleFactor = 0.4;
        
        reset();
        }
    
    public void paint(Graphics g) {
        width = getWidth();
        height = getHeight();
        
        g.setColor(Color.WHITE);
        g.fillRect(0,0,width,height);
        if(enabled) {
            for(int i=0;i<16;i++) {
                double scaledSonar = sonars[i]*scaleFactor;
                double scaledX = x[i]*scaleFactor;
                double scaledY = y[i]*scaleFactor;
                int x1 = (int)(width/2 + scaledX);
                int y1 = (int)(height/2 - scaledY);
                int x2 = (int)(x1+scaledSonar*Math.cos(angles[i]+spreadAngle));
                int y2 = (int)(y1-scaledSonar*Math.sin(angles[i]+spreadAngle));
                int x3 = (int)(x1+scaledSonar*Math.cos(angles[i]-spreadAngle));
                int y3 = (int)(y1-scaledSonar*Math.sin(angles[i]-spreadAngle));

                g.setColor(Color.decode("#009900"));
                g.drawLine(x1,y1,x2,y2);
                g.drawLine(x2,y2,x3,y3);
                g.drawLine(x3,y3,x1,y1);

                //double xTanTheta = sonars[i]*Math.tan(spreadAngle);
                //g.drawArc((int)((x2+x3)/2-xTanTheta), (int)((y2+y3)/2-xTanTheta), (int)(2*xTanTheta)+2, (int)(2*xTanTheta)+2,
                //          (int)((angles[i]*180/Math.PI)-90), 180);
                }
            } else {
            g.setColor(Color.RED);
            g.drawLine(0,0,getWidth(),getHeight());
            g.drawLine(getWidth(), 0, 0, getHeight());
            }
        g.setColor(Color.BLACK);
        g.drawRect(0,0,width-1,height-1);
        }
    
    public void reset() {
        for(int i=0;i<16;i++) {
            sonars[i] = 50.0;
            }
        angles[0] = Math.PI;
        angles[7] = 0;
        angles[8] = 0;
        angles[15] = Math.PI;
        for(int i=1;i<=6;i++) {
            angles[i] = Math.PI - Math.PI/9*(i+1);
            }
        for(int i=9;i<=14;i++) {
            angles[i] = 2*Math.PI-(Math.PI/9*(i-8));
            }
        for(int i=0;i<16;i++) {
            x[i] = (int)(140*Math.cos(angles[i]));
            if(i<8) {
                y[i] = (int)(107+140*Math.sin(angles[i]));
                } else {
                y[i] = (int)(-107+140*Math.sin(angles[i]));
                }
            }
        spreadAngle = Math.PI/18;
        }
    public void setEnabled(boolean set) {
        enabled = set;
        }
    public boolean getEnabled() {
        return(enabled);
        }
    }
