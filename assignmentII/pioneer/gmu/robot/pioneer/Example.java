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

	public static int GOAL_X = 900;
	public static int GOAL_Y = 0;

	static short[] sonar_angles = {90, 50, 30, 10, -10, -30, -50, -90, -90, -130, -150, -170, 170, 150, 130, 90};
	
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
        //testFunc(robot, f);
        try {
        	System.out.println("Sleeping");
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        robot.seto();
        double[] sensors = f.getFilteredSonarValues();
        
        double radTheta = getHeadingFromZero();
    	double i = 0;
    	double j = 0;
        int c = 3;
        
        // if I'm not at goal
        while (((Math.abs(GOAL_X - robot.getXPos()) > 100) ||
        		((Math.abs(GOAL_Y - robot.getYPos())) > 100))) 
        {
        	double att_x = GOAL_X - robot.getXPos();
        	double att_y = GOAL_Y - robot.getYPos();

        	double frepMag = 0;
        	double[] frepComp = new double[2];
        	frepComp[0] = 0;
        	frepComp[1] = 0;
        	
            sensors = f.getFilteredSonarValues();
        	for (int k = 0; k < 16; ++k) {
        		if (sensors[k] < 300) {
        			System.out.println("LESS THAN 100");
        			frepMag = sensors[k];
        			double theta = robot.getOrientation() + Math.toRadians(sonar_angles[k]);
        			
        			double x_obs = (frepMag) * Math.cos(theta);
        			double y_obs = (frepMag) * Math.sin(theta);
        			
        			frepComp[0] = (x_obs - robot.getXPos());
        			frepComp[1] = (y_obs - robot.getYPos());
        			
        			System.out.println("***********************SAW OBJ at " + k);
        			System.out.println("***********************SAW OBJ at " + k);
        			System.out.println("***********************SAW OBJ at " + k);
        			System.out.println("***********************SAW OBJ at " + k);
        			System.out.println("***********************SAW OBJ at " + k);
        			System.out.println("***********************SAW OBJ at " + k);
        			break;
        		}
        	}
        	
        	double[] f_sum = new double[2];
        	f_sum[0] = att_x + frepComp[0];
        	f_sum[1] = att_y + frepComp[1];
 
        	i = f_sum[0];
        	j = f_sum[0] + (2 * f_sum[1]);
        	
        	System.out.println("IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII " + i);
        	System.out.println("JJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJ " + j);
        	System.out.println("AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");
        	
        	double max = 5;
        	if (Math.abs(i) > max || Math.abs(j) > max) {
        		if (i >= j) {
        			double temp = i;
        			i *= max / Math.abs(temp);
        			j *= max/ Math.abs(temp);
        		} else {
        			double temp = i;
        			i *= max / Math.abs(temp);
        			j *= max/ Math.abs(temp);
        		}
        	}
        	
        	System.out.println("IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII " + i);
        	System.out.println("JJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJ " + j);
        	
        	while (Math.abs(i) > 13 || Math.abs(j) > 13) {
        		System.out.println("************************ DROPPING VALUES ********************* ");
        		i /= 1.1;
        		j /= 1.1;
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
    
    public static double getHeadingFromRandomFrickenRobot(PioneerRobot robot) {
    	double vecX = GOAL_X - robot.getXPos();
    	double vecY = GOAL_Y - robot.getYPos();
    	
    	return Math.atan(vecY / vecX);
    	
    }
    
    public static double getHeadingFromFrickenPoint(double[] p_points) {
    	double vecX = GOAL_X - p_points[0];
    	double vecY = GOAL_Y - p_points[1];
    	
    	return Math.atan(vecY / vecX);
    }
    
    public static void testFunc(PioneerRobot robot, MedianFilter f) {
    	double frepMag = 0;
    	double[] arr = new double[2];
        double[] sensors = f.getFilteredSonarValues();
    	for (int k = 0; k < 16; ++k) {
    		if (sensors[k] < 300) {
    			System.out.println("LESS THAN 100");
    			frepMag = sensors[k];
    			double theta = robot.getOrientation() + Math.toRadians(sonar_angles[k]);
    			
    			double x_obs = frepMag * Math.cos(theta);
    			double y_obs = frepMag * Math.sin(theta);
    			
    			arr[0] = x_obs - robot.getXPos();
    			arr[1] = y_obs - robot.getYPos();
    		}
    	}
    }
    }
