/*
 * PioneerGui.java
 *
 * By Joey Harrison
 * Last Modified 12/8/07
 * 
 * This GUI has a main map window which displays the Pioneer, a joystick panel
 * which allows the Pioneer to be joysticked, and some extra controls.
 * It also maintains a Pioneer robot and monitors its position to update the map
 * controls.
 */
package gmu.robot.pioneer.jgui;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.geom.Point2D;
import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Iterator;

import gmu.robot.pioneer.PioneerRobot;
import gmu.robot.pioneer.jgui.portrayals.PioneerPortrayal;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JPanel;

public class PioneerGui extends JFrame implements Runnable
    {
    private static final long serialVersionUID = 1L;
        
    PioneerRobot robot;
    MapPanel mapPanel;
    JPanel controlPanel;
    JoystickPanel joystickPanel;
    PioneerPortrayal pioneerPortrayal;

    JPanel buttonPanel;
    JButton btnFollowPioneer;
    JButton btnClearPathHistory;
    JButton btnSavePathHistory;
        
    public PioneerGui(int port)
        {
        try
            {
            robot = new PioneerRobot();
            robot.connect("10.0.0.101", port);
            robot.sonar(false);
            }
        catch (Exception e) { e.printStackTrace(); }
                
        addWindowListener( new WindowAdapter() { 
            public void windowClosing(WindowEvent e) { 
                if(robot != null)
                    {
                    robot.disconnect();
                    robot.close();
                    }
                System.exit(0); 
                } 
            });
        
        // start adding visual components

        setTitle("Pioneer GUI");
        setVisible(true);
        this.setLayout(new BorderLayout());
        
        // set up the map panel 
        mapPanel = new MapPanel();
        mapPanel.setPreferredSize(new Dimension(640, 480));
        
        pioneerPortrayal = new PioneerPortrayal();
        mapPanel.mapDisplay.portrayals.add(pioneerPortrayal);
        
        add(mapPanel, BorderLayout.CENTER);

        // set the control panel up on the right side 
        controlPanel = new JPanel();
        controlPanel.setPreferredSize(new Dimension(240, 480));
        controlPanel.setLayout(new BorderLayout());
        
        add(controlPanel, BorderLayout.EAST);
        
        joystickPanel = new JoystickPanel(robot);
        joystickPanel.setBorder(BorderFactory.createTitledBorder(" Joystick "));
        joystickPanel.setPreferredSize(new Dimension(280, 260));
        controlPanel.add(joystickPanel, BorderLayout.NORTH);
        
        buttonPanel = new JPanel();
        buttonPanel.setLayout(new BoxLayout(buttonPanel, BoxLayout.Y_AXIS));
        
        controlPanel.add(buttonPanel, BorderLayout.SOUTH);
        
        btnFollowPioneer = new JButton("Follow Pioneer");
        btnFollowPioneer.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e)
                {
                mapPanel.mapDisplay.setObjectToFollow(pioneerPortrayal);
                mapPanel.repaint();
                }
            });
        
        btnClearPathHistory = new JButton("Clear Path History");
        btnClearPathHistory.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e)
                {
                pioneerPortrayal.clearPathHistory();
                //TODO: make a way to do the repaint from inside the portrayal.
                // It might need to keep a reference to its containing panel.
                mapPanel.repaint();
                }
            });
                
        btnSavePathHistory = new JButton("Save Path History");
        btnSavePathHistory.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e)
                {
                JFileChooser jc = new JFileChooser();
                        
                jc.showSaveDialog(buttonPanel);
                savePathHistory(jc.getSelectedFile());
                        
                }
            });
                

        buttonPanel.add(Box.createVerticalGlue());
        buttonPanel.add(Box.createRigidArea(new Dimension(0, 10)));
        buttonPanel.add(btnFollowPioneer);
        buttonPanel.add(btnClearPathHistory);
        buttonPanel.add(btnSavePathHistory);
        buttonPanel.add(Box.createRigidArea(new Dimension(20, 20)));
        
        // start the loop to monitor robot position
        Thread t = new Thread(this);
        t.start();
        
        pack();
        repaint();

        // set up to start out zoomed out a little and centered on the pioneer
        btnFollowPioneer.doClick();
        mapPanel.mapDisplay.zoom(2.0);
        }
        
    /*
     * Save the path history to the given file. The path will be saved in CSV format:
     * x1, y1
     * x2, y2
     * .
     * .
     * .
     * 
     * And so on.
     */
    private void savePathHistory(File f)
        {
        if (f == null)
            return;
                
        try
            {
            PrintStream out = new PrintStream(new FileOutputStream(f));
                        
            ArrayList<Point2D.Double> points = pioneerPortrayal.getPathHistoryPoints();
                        
            Iterator<Point2D.Double> iter = points.iterator();
            Point2D.Double p;
                        
            while (iter.hasNext())
                {
                p = iter.next();
                out.format("%.4f, %.4f\n", p.x, p.y);
                }
                        
            out.close();
            }
        catch (Exception e) { e.printStackTrace(); }
        }

    /*
     * Run a loop to monitor the pose of the robot. When the pose changes,
     * it adds the new pose to the path history and updates the screen.
     */
    public void run()
        {
        double x, y, yaw;
        double prevX=0, prevY=0, prevYaw=0;
        try
            {
            while (true)
                {
                x = robot.getXPos();
                y = robot.getYPos();
                yaw = robot.getOrientation();
                                
                // only update if the point has changed
                if ((x != prevX) || (y != prevY) || (yaw != prevYaw))
                    {
                    //System.out.format("x: %.1f, y: %.1f, yaw: %.1f\n", 
                    //              robot.getXPos(), robot.getYPos(), 
                    //              robot.getOrientation() * 180.0 / Math.PI);
                                        
                    pioneerPortrayal.setPose(-y, x, yaw);   // to the robot, x is forward and y is left
                    mapPanel.repaint();
                    }
                                
                prevX = x;
                prevY = y;
                prevYaw = yaw;
                                
                Thread.sleep(50);
                }
            }
        catch (Exception ex) { ex.printStackTrace(); }
        }

    public static void main(String[] args)
        {
        int port = 5000;
                
        if (args.length > 1)
            port = Integer.parseInt(args[0]);
                
        PioneerGui gui = new PioneerGui(port);
        }
    }
