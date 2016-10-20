package gmu.robot.pioneer;
import javax.swing.*;
import java.awt.event.*;
import java.awt.*;
import java.io.*;

/* Sean */
public class Joystick extends JPanel
    {
    gmu.robot.pioneer.PioneerRobot robot;
        
    boolean stopped = true;
    int xPos;
    int yPos;
        
    public void paintComponent(Graphics g)
        {
        g.setColor(Color.white);
        g.fillRect(0,0,getWidth(), getHeight());
        if (stopped)
            {
            g.setColor(Color.red);
            xPos = getWidth() / 2;
            yPos = getHeight() / 2;
            }
        else
            g.setColor(Color.green);
        g.fillOval(xPos - getWidth() / 20, yPos - getHeight() / 20,
            getWidth() / 10, getHeight() / 10);
        }
                
    void setMotors(MouseEvent e)
        {
        double x = (1.0 - e.getX() / (double)getWidth()) * 2 - 1;
        double y = (1.0 - e.getY() / (double)getHeight()) * 2 - 1;
        double left = (y - x) / 2;
        double right = (y + x) / 2;
        robot.vel2((byte)(((left) * 127) / 4), (byte)(((right) *127) / 4));
        stopped = false;
        xPos = e.getX();
        yPos = e.getY();
        repaint();
        }
        
    public Joystick(final gmu.robot.pioneer.PioneerRobot robot) throws IOException
        {
        this.robot = robot;
                
        addMouseMotionListener(new MouseMotionAdapter()
            {
            public void mouseDragged(MouseEvent e)
                {
                setMotors(e);
                }
            });
                        
        addMouseListener(new MouseAdapter()
            {
            public void mousePressed(MouseEvent e)
                {
                setMotors(e);
                }
                                
            public void mouseReleased(MouseEvent e)
                {
                stopped = true;
                robot.vel2((byte)0,(byte)0);
                repaint();
                }
            });
        }

    public static void usage()
        {
        System.out.println("Usage: Joystick <port>");
        System.exit(0);
        }

    public static void main(String[] args) throws Exception
        {
        if (args.length<1)
            usage();

        final gmu.robot.pioneer.PioneerRobot robot = new gmu.robot.pioneer.PioneerRobot();

        robot.setVerbose(true);
        robot.connect(args[0], Integer.parseInt(args[1]));
        robot.setVerbose(true);
        robot.enable(true);
        robot.sonar(true);
        Joystick joystick = new Joystick(robot);
        JFrame frame = new JFrame();
        frame.getContentPane().setLayout(new BorderLayout());
        frame.getContentPane().add(joystick, BorderLayout.CENTER);
                
        Box box = Box.createHorizontalBox();
        final JCheckBox sonar = new JCheckBox("Sonar", true);
        sonar.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e)
                { robot.sonar(sonar.isSelected()); }
            });
        box.add(sonar);
        final JCheckBox motors = new JCheckBox("Motors", true);
        sonar.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e)
                { robot.enable(motors.isSelected()); }
            });
        box.add(motors);
        box.add(box.createGlue());
        frame.getContentPane().add(box, BorderLayout.SOUTH);
        frame.pack();
        frame.setSize(300,300);
        frame.setVisible(true);
        }
    }
