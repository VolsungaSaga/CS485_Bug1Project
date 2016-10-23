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

	public static final int GOAL_X = 15;
	public static final int GOAL_Y = 0;
	
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
        MedianFilter f = new MedianFilter(robot);
        
        robot.setVerbose(true);
        robot.connect("127.0.0.1", Integer.parseInt(args[0]));
        robot.sonar( true );
        robot.enable( true );
        robot.setVerbose(true); 
        try {
        	System.out.println("Sleeping");
			Thread.sleep(10000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        //robot.vel2((byte)10, (byte)10);
        robot.seto();
        double[] sensors = f.getFilteredSonarValues();
        
        double radTheta = getHeading();
        short degrees = (short)Math.floor(Math.toDegrees(radTheta));
          
        //if I'm not at goal
        while ((robot.getXPos() != GOAL_X || robot.getYPos() != GOAL_Y) || 
        		((Math.abs(GOAL_X - robot.getXPos()) < 1 ) ||
        		((Math.abs(GOAL_Y - robot.getYPos())) < 1))) 
        {
        	robot.vel2((byte)10, (byte)10);
        	System.out.println("------------------- ROBOT X " + robot.getXPos());
        	System.out.println("------------------- ROBOT Y " + robot.getYPos());
        }         
        try { 
            Thread.sleep(5000);
            } catch (Exception e) {}
                robot.e_stop(); 
        robot.enable(false); 
        System.out.println(robot.getXPos()); 
        try { 
            Thread.sleep(2000); } catch (Exception e) {} 
        
        System.out.println(robot.getXPos()); 

        robot.disconnect();
        }
    
    /**
     * @return theta that puts us inline with the goal
     */
    public static double getHeading() {
    	return  Math.acos( GOAL_X / (Math.sqrt((GOAL_X * GOAL_X) + (GOAL_Y * GOAL_Y))) );
    }
    
    }
