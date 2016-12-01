/*
 * SonarPanel.java
 *
 * Created on October 11, 2006, 9:00 PM
 *
 */

package gmu.robot.pioneer.gui;

import javax.swing.*;
import javax.swing.event.*;
import java.awt.*;
import java.awt.event.*;
import gmu.robot.pioneer.*;

public class SonarPanel extends JPanel implements ChangeListener {

    SonarDisplay display;
    JCheckBox enable;
    JSlider sonarScaleSlider;
    JLabel sonarScaleSliderLabel;
    JSlider sonarSpreadSlider;
    JLabel sonarSpreadSliderLabel;
    JTextField[] sonarValues;
    gmu.robot.pioneer.PioneerRobot bot;
    
    public volatile double[] sonars;
    
    public SonarPanel(gmu.robot.pioneer.PioneerRobot robot) {
        bot = robot;
        
        sonarScaleSliderLabel = new JLabel();
        sonarScaleSliderLabel.setText("Scale");
        sonarScaleSliderLabel.setBounds(215,10,150,25);
        add(sonarScaleSliderLabel);
        
        sonarScaleSlider = new JSlider();
        sonarScaleSlider.setOrientation(JSlider.HORIZONTAL);
        sonarScaleSlider.setBounds(210,30,250,20);
        sonarScaleSlider.setMinimum(1);
        sonarScaleSlider.setMaximum(200);
        sonarScaleSlider.setValue(100);
        sonarScaleSlider.addChangeListener(this);
        add(sonarScaleSlider);

        sonarSpreadSliderLabel = new JLabel();
        sonarSpreadSliderLabel.setText("Spread");
        sonarSpreadSliderLabel.setBounds(215,45,150,25);
        add(sonarSpreadSliderLabel);
        
        sonarSpreadSlider = new JSlider();
        sonarSpreadSlider.setOrientation(JSlider.HORIZONTAL);
        sonarSpreadSlider.setBounds(210,65,250,20);
        sonarSpreadSlider.setMinimum(0);
        sonarSpreadSlider.setMaximum(500);
        sonarSpreadSlider.setValue(250);
        sonarSpreadSlider.addChangeListener(this);
        add(sonarSpreadSlider);        
        
        sonars = new double[16];
        for(int i=0;i<16;i++) {
            sonars[i] = 0;
            }
        
        display = new SonarDisplay(sonars);
        display.setBounds(10,20,200,200);
        add(display);
        
        enable = new JCheckBox();
        enable.setBounds(10,220,100,30);
        enable.setText("Enable sonar");
        enable.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                display.setEnabled(enable.isSelected());
                bot.sonar(enable.isSelected());
                }
            });
        add(enable);
        
        sonarValues = new JTextField[16];
        for(int i=0;i<8;i++) {
            sonarValues[i] = new JTextField();
            sonarValues[i].setText(String.valueOf(i)+": ");
            sonarValues[i].setBounds(220,85+20*i,115,20);
            sonarValues[i].setEditable(false);
            add(sonarValues[i]);
            }
        for(int i=8;i<16;i++) {
            sonarValues[i] = new JTextField();
            sonarValues[i].setText(String.valueOf(i)+": ");
            sonarValues[i].setBounds(345,85+20*(i-8),115,20);
            sonarValues[i].setEditable(false);
            add(sonarValues[i]);
            }
        updateSonarLabels();
        
        setSize(470,270);
        repaint();
        }

    public void stateChanged(ChangeEvent e) {
        Object o = e.getSource();
        if(o==sonarScaleSlider) {
            display.scaleFactor = (double)(sonarScaleSlider.getValue())/2000;
            } else if(o==sonarSpreadSlider) {
            display.spreadAngle = (double)(sonarSpreadSlider.getValue())*(Math.PI/12)/500;
            }
        repaint();
        }
    public void setSonarArray(double[] newArray) {
        if(newArray.length>=16)
            sonars = newArray;
        updateSonarLabels();
        }
    public void setSonarValues(double[] valArray) {
        if(valArray.length>=16) {
            for(int i=0;i<16;i++) {
                sonars[i] = valArray[i];
                }
            }
        updateSonarLabels();
        }
    private void updateSonarLabels() {
        for(int i=0;i<16;i++) {
            String text = String.valueOf(i)+": "+String.valueOf(sonars[i]);
            if(text.length()>18) {
                text = text.substring(0,17);
                }
            sonarValues[i].setText(text);
            }
        }
    }
