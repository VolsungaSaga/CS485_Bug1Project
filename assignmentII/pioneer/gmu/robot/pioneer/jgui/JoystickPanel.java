/*
 * JoystickPanel.java
 *
 * Created on October 18, 2006, 1:05 PM
 * By Sean Robinette
 * 
 * Spruced up by Joey Harrison  
 * Last Modified 12/8/07
 */

package gmu.robot.pioneer.jgui;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

public class JoystickPanel extends JPanel
    {   
    private static final long serialVersionUID = 1L;
    private gmu.robot.pioneer.PioneerRobot bot;
    private boolean enabled = false;
    
    JPanel leftPanel;
    JoystickDisplay display;
    JCheckBox enableMotors;
    JSlider speedSlider;
        
    public JoystickPanel(gmu.robot.pioneer.PioneerRobot robot) 
        {
        bot = robot;

        setLayout(new BorderLayout());
        
        display = new JoystickDisplay(bot);
        display.setPreferredSize(new Dimension(200, 200));
        
        add(display, BorderLayout.CENTER);
        
        enableMotors = new JCheckBox("Enable");
        enableMotors.setPreferredSize(new Dimension(80, 20));
        enableMotors.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                enabled = enableMotors.isSelected();
                bot.enable(enabled);
                display.setEnabled(enabled);
                }
            });

        add(enableMotors, BorderLayout.SOUTH);
        
        speedSlider = new JSlider();
        speedSlider.setMinimum(10);
        speedSlider.setMaximum(1400);
        speedSlider.setOrientation(JScrollBar.VERTICAL);
        speedSlider.setPreferredSize(new Dimension(20, 200));
        speedSlider.addChangeListener(new ChangeListener()
            {
            public void stateChanged(ChangeEvent arg0)
                {
                bot.setv((short)speedSlider.getValue());                                
                }
            });

        add(speedSlider, BorderLayout.EAST);
        
        // set this here so it sends the setv command
        speedSlider.setValue(400);
        }
    }
