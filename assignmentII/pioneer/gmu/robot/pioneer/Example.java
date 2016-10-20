/*
 * Example.java
 */

package gmu.robot.pioneer;

/**
 * This is the sample program for controlling an AmigoBot robot.
 * <P>
 * <A HREF="../COPYRIGHT.html">Copyright</A>
 * (c)2001 Evolutinary Computation Laboratory @ George Mason University
 *
 * @author Liviu Panait
 * @version 1.0
 */

public class Example
    {

    public static void usage()
        {
        System.out.println("Usage: Example <port>");
        System.exit(0);
        }

    public static final void main( String[] args ) throws Exception
        {
        if (args.length<1)
            usage();

        PioneerRobot robot = new PioneerRobot();
        robot.setVerbose(true);
        robot.connect("10.0.0.133", Integer.parseInt(args[0]));
        robot.sonar( false );
        robot.enable( true );
        robot.setVerbose(false); 
        
        robot.vel2((byte)10, (byte)10); 
        try { 
            Thread.sleep(5000);
            } catch (Exception e) {}

        robot.e_stop(); 
        robot.enable(false); 
        System.out.println(robot.getXPos()); 
        try { 
            Thread.sleep(2000); } catch (Exception e) {} 
        //robot.resetOdometery(); 
        System.out.println(robot.getXPos()); 
        
        /* for( short i = 0 ; i < 10 ; i++ )
           {
           robot.sound(i);
           robot.dhead( (short)(60 * (short)(2*(i%2)-1)) );
           Thread.sleep(1000);
           }
        */
        robot.disconnect();
        }
    }
