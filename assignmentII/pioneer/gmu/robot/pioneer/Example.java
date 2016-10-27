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

	public static int GOAL_X = 300;
	public static int GOAL_Y = 300;
	
    public static void usage()
        {
        System.out.println("Usage: Example <port>");
        System.exit(0);
        }

    public static final void main( String[] args ) throws Exception
        {
//        double radTheta1 = getHeading();
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
        robot.seto();
        double[] sensors = f.getFilteredSonarValues();
        
        double radTheta = getHeadingFromZero();
    	double i = 3;
    	double j = 3;
        
        //if I'm not at goal
        while (((Math.abs(GOAL_X - robot.getXPos()) > 50) ||
        		((Math.abs(GOAL_Y - robot.getYPos())) > 50))) 
        {
        	if ((robot.getOrientation() - radTheta) > 0.1) {
        		i += 0.0001;
        		j -= 0.0001;
        	} else if ((robot.getOrientation() - radTheta) < -0.1) {
        		i -= 0.0001;
        		j += 0.0001;
        	} else {
        		System.out.println("******** Hit else statement ***********");
        		i = 3;
        		j = 3;
        		
        		if ((Math.abs(GOAL_X - robot.getXPos()) < 50) || 
        				(Math.abs(GOAL_Y - robot.getYPos()) < 50)) {
        			System.out.println("************************* HIT IF STATEMENT");
        			System.out.println("************************* HIT IF STATEMENT");
        			System.out.println("************************* HIT IF STATEMENT");
        			System.out.println("************************* HIT IF STATEMENT");
        			radTheta = getHeadingFromRandomFrickenPoint(robot);
        		} 
        	}
        	
   			robot.vel2((byte)i, (byte)j);
    		Thread.sleep(1000);
    		
   			System.out.println("-----------------------sRad theta " + radTheta);
        	System.out.println("------------------- ROBOT X " + robot.getXPos());
        	System.out.println("------------------- ROBOT Y " + robot.getYPos());
        	System.out.println("------------------- ROBOT theta " + robot.getOrientation());
        }
      System.out.println("**************************** Broke out of while ******************************");
      System.out.println("**************************** Broke out of while ******************************");
      System.out.println("**************************** Broke out of while ******************************");
      System.out.println("**************************** Broke out of while ******************************");
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
    public static double getHeadingFromZero() {
    	double x_rad = Math.acos( GOAL_X / (Math.sqrt((GOAL_X * GOAL_X) + (GOAL_Y * GOAL_Y))) );
    	double y_rad = Math.asin( GOAL_Y / (Math.sqrt((GOAL_X * GOAL_X) + (GOAL_Y * GOAL_Y))) );
    	if (GOAL_X > 0 && GOAL_Y > 0) {
    		return  x_rad;
    	} else if (GOAL_X > 0 && GOAL_Y < 0) {
    		return y_rad;
    	} else if (GOAL_X < 0 && GOAL_Y > 0) {
    		return x_rad;
    	} else {
    		return y_rad;
    	}
    }
    
    public static double getHeadingFromRandomFrickenPoint(PioneerRobot robot) {   	
    	double vecX = GOAL_X - robot.getXPos();
    	double vecY = GOAL_Y - robot.getYPos();
    	
    	return Math.atan(vecY / vecX);
    }
    
    }
