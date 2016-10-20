/*
 * JoystickPanel.java
 *
 * Created on October 18, 2006, 1:05 PM
 * By Sean Robinette
 */

package gmu.robot.pioneer.gui;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import gmu.robot.pioneer.*;

public class JoystickPanel extends JPanel {
    
    private gmu.robot.pioneer.PioneerRobot bot;
    private boolean enabled = false;
    
    JoystickDisplay display;
    JCheckBox enableMotors;
    JCheckBox useMouse;
    JSlider speedSlider;
    Bar leftWheel;
    Bar rightWheel;
    RotationDisplay rotate;
        
    public void setPos(double x, double y) {
        display.setPos(x,y);
        }
    public double getXPos() {
        return(display.getXPos());
        }
    public double getYPos() {
        return(display.getYPos());
        }
    public boolean isUsingMouse() {
        return(useMouse.isSelected());
        }
    public boolean isMotorEnabled() {
        return(enabled);
        }
    public JoystickPanel(gmu.robot.pioneer.PioneerRobot robot) {
        bot = robot;
        
        display = new JoystickDisplay(bot);
        display.setBounds(0,0,200,200);
        add(display);
        enableMotors = new JCheckBox("Motors");
        enableMotors.setBounds(10,210,80,20);
        enableMotors.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                enabled = enableMotors.isSelected();
                bot.enable(enabled);
                display.setEnabled(enabled);
                }
            });
        add(enableMotors);        
        useMouse = new JCheckBox("Mouse");
        useMouse.setBounds(100,210,80,20);
        useMouse.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                display.setMouse(useMouse.isSelected());
                display.setPos(0,0);
                }
            });
        add(useMouse);        
        
        leftWheel = new Bar();
        leftWheel.setBounds(230, 0, 100, 15);
        add(leftWheel);
        
        rightWheel = new Bar();
        rightWheel.setBounds(230, 20, 100, 15);
        add(rightWheel);
        
        rotate = new RotationDisplay();
        rotate.setBounds(230, 40, 100, 100);
        add(rotate);
        
        speedSlider = new JSlider();
        speedSlider.setMinimum(1);
        speedSlider.setMaximum(12);
        speedSlider.setOrientation(JScrollBar.VERTICAL);
        speedSlider.setBounds(210, 0, 20, 200);
        add(speedSlider);
        }
    public void updateProperties() {
        leftWheel.setValue(bot.getLeftWheelVelocity()/(20*speedSlider.getValue()) + 0.5);
        rightWheel.setValue(bot.getRightWheelVelocity()/(20*speedSlider.getValue()) + 0.5);        
        rotate.setAngle(3*Math.PI/2 - bot.getOrientation());
        }
    public int getSpeed() {
        return(speedSlider.getValue());
        }
    }
