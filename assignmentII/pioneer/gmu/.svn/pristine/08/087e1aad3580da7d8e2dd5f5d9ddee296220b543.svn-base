/*
  Requires wiiremotej and bluecove.  See the libraries directory.
*/

package gmu.robot.pioneer;
import java.io.*;
import wiiremotej.*;
import wiiremotej.event.*;

public class WiimoteRobot extends WiiRemoteAdapter
    {
    public static void main(String[] args) throws Exception
        {
        // turn off a stupid bug in bluecove
        System.setProperty("bluecove.jsr82.psm_minimum_off", "true");

        // remind the user that he must run in 32-bit mode on OS X
        System.out.println("Note you must use WiimoteRobot in -d32 (supported by Java 1.5 but not 1.6 on OS X)");
        System.out.println("Press 1 and 2 on your wiimote");
        WiiRemote mote = WiiRemoteJ.findRemote();
        
        mote.setLEDIlluminated(3, true);
        mote.setAccelerometerEnabled(true);
        
        PioneerRobot therobot = new PioneerRobot();
        therobot.connect("localhost", 5000);
        therobot.enable(true);
        therobot.setVerbose(true);

        mote.addWiiRemoteListener(new WiimoteRobot(therobot));
        }
        
    public PioneerRobot robot;
    boolean enabled = false;
    boolean buttonsOff = true;
    byte MAX_SPEED = 30;
        
    public WiimoteRobot(PioneerRobot robot)
        {
        this.robot = robot;
        }
        
    public void buttonInputReceived(WRButtonEvent e)
        {
        if (e.isPressed(WRButtonEvent.B) && !enabled)
            {
            enabled = true;
            }
        else if (!e.isPressed(WRButtonEvent.B) && enabled)
            {
            enabled = false;
            robot.vel2((byte)0,(byte)0);
            }
            
        if (e.isPressed(WRButtonEvent.UP))
            {
            if (buttonsOff) 
                {
                MAX_SPEED += 10;
                if (MAX_SPEED > 60) MAX_SPEED = 60;
                System.out.println(MAX_SPEED);
                }
            buttonsOff = false;
            }
        else if (e.isPressed(WRButtonEvent.DOWN))
            {
            if (buttonsOff) 
                {
                MAX_SPEED -= 10;
                if (MAX_SPEED < 0 ) MAX_SPEED = 0;
                System.out.println(MAX_SPEED);
                }
            buttonsOff = false;
            }
        else buttonsOff = true;
        }
            
    public void accelerationInputReceived(WRAccelerationEvent e)
        {
        if (e.isStill())
            {
            double pitch = e.getPitch();
            if (pitch < -Math.PI/4) pitch = -Math.PI/4;
            if (pitch > Math.PI/4) pitch = Math.PI/4; 
            double forward = (-pitch) * MAX_SPEED / (Math.PI/4);
            double rot = e.getRoll();
            if (rot > Math.PI) rot -= 2 * Math.PI;
            if (rot > Math.PI/2) rot = Math.PI/2;
            if (rot < -Math.PI/2) rot = -Math.PI/2;
            double turn = rot * MAX_SPEED / (Math.PI/2);
                
            robot.vel2((byte)((forward + turn)*(enabled ? 1 : 0)),
                (byte)((forward - turn)*(enabled ? 1 : 0)));
            }
        }
    }
