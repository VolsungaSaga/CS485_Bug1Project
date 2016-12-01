/*
 * AmigoBotSerial.java
 */

package gmu.robot.pioneer.serial;

import gmu.robot.pioneer.*;
import java.io.*;

/**
 * This is the main class for controlling an AmigoBot robot. It has functions
 * for everything from connecting to/disconnecting from the robot,
 * setting speed and heading, to managing the pulse beat (such that the robot
 * receives some commands and does not think it got disconnected) and reading
 * information from the robot.
 * <P>
 * <A HREF="../COPYRIGHT.html">Copyright</A>
 * (c)2001 Evolutinary Computation Laboratory @ George Mason University
 *
 * @author Liviu Panait
 * @version 1.0
 */

public class RobotSerial extends PioneerRobot
    {
    SerialCommunication sc;

    /** connect to the robot */
    public void connect(String serialPortDevice) throws IOException
        {
        sc = new SerialCommunication(serialPortDevice);  // "/dev/tty.USA19QW181P1.1"
        connect(sc.getInputStream(),sc.getOutputStream());
        }

    public void disconnect() throws IOException
        {
        super.disconnect();
        sc.shutDown();
        }

    }
