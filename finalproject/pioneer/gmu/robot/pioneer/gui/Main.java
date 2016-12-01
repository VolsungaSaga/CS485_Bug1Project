/*
 * Main.java
 *
 * Created on October 11, 2006, 6:06 PM
 * by Sean Robinette
 */

package gmu.robot.pioneer.gui;

import java.io.*;
import java.util.*;
//import javax.comm.*;
import javax.swing.*;
import javax.swing.event.*;
import java.awt.event.*;
//import gmu.robot.pioneer.serial.*;
import gmu.robot.pioneer.*;
//import de.hardcode.jxinput.*;

public class Main extends JFrame {
    
    //RobotSerial bot;
    PioneerRobot bot;
    SonarPanel sonar;
    JCheckBox enableMotors;
    JCheckBox enableSonar;
    JoystickPanel joy;
    //JXInputDevice input;
    //Axis axes[];
    
    int maxSpeed = 8;
    
    long timeStamp;
    long loopCount = 0;
    
    public Main() {
        //bot = new RobotSerial();
        bot = new PioneerRobot();
        
        addWindowListener( new WindowAdapter() { 
            public void windowClosing(WindowEvent e) { 
                if(bot!=null)
                    bot.close();
                System.exit(0); 
                } 
            });
        
        setTitle("PioneerRobot GUI");
        setBounds(50,50,500,700);
        setResizable(false);
        setVisible(true);
        
        sonar = new SonarPanel(bot);
        sonar.setLocation(20,20);
        sonar.setBorder(javax.swing.BorderFactory.createTitledBorder(" Sonar Display "));
        add(sonar);
        
        
/*
  if(JXInputManager.getNumberOfDevices()>0) {
  input = JXInputManager.getJXInputDevice(0);
  if(input.getNumberOfAxes()>=2) {
  axes = new Axis[2];
  axes[0] = input.getAxis(0);
  axes[1] = input.getAxis(1);                
  }
  }
*/                
        joy = new JoystickPanel(bot);
        joy.setBounds(10,400,400,300);
        add(joy);
                
        repaint();
        
        // Connect to the robot
        /*
          Enumeration portIdentifiers = CommPortIdentifier.getPortIdentifiers();
          while (portIdentifiers.hasMoreElements())
          {
          CommPortIdentifier pid = (CommPortIdentifier) portIdentifiers.nextElement();
          System.err.println(pid.getName());
          }
        */

        //try {
        bot.setVerbose(true);
        bot.connect("localhost", 5000);
        //bot.connect("COM6");
        //}  catch(IOException err) { }
        bot.sonar(false);
        bot.enable(false);
        /*bot.say(new byte[]{(byte)30, (byte)69,
          (byte)15, (byte)67,
          (byte)15, (byte)65,
          (byte)15, (byte)63,
          (byte)15, (byte)69,
          (byte)15, (byte)67,
          (byte)15, (byte)65,
          (byte)30, (byte)67,
          (byte)30, (byte)65,
          (byte)60, (byte)67});*/
        }
    public void loop() {
        timeStamp = System.currentTimeMillis();
        while(true) {
            timeStamp = System.currentTimeMillis();
            while((System.currentTimeMillis()-timeStamp)<10) {}
            sonar.setSonarValues(bot.getSonars());
            /*
              if(!joy.isUsingMouse() && joy.isMotorEnabled()) {
              if(input!=null) {
              JXInputManager.updateFeatures();
              double x1 = axes[0].getValue();
              double y1 = axes[1].getValue();
              joy.setPos(x1,y1);
              byte x = (byte)(-(y1-x1)*maxSpeed);
              byte y = (byte)(-(y1+x1)*maxSpeed);
              bot.vel2(x,y);
              }
              }
            */
            joy.updateProperties();
            maxSpeed = joy.getSpeed();
            try { Thread.sleep(100); } catch (InterruptedException e) { }
            repaint();
            }
        }
    public static void main(String[] args) {
        Main theApp = new Main();
        theApp.loop();
        }
    
    }

