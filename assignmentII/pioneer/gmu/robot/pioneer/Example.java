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

   //INITIALIZATION
        
        //According to the Pioneer 3's operator manual, these are the angles at 
    	// which sonars 0 - 7 are located. I don't know if these mesh with the Pioneer's 
    	// internal understanding of its own orientation, but nothing ventured, nothing gained.
    	
    	/*		SonarNum:		0    1    2    3    4  5   6   7 */
    	short[] sonarAngles = {270, 310, 330, 350, 10, 30, 50, 90};
    	
    	double attractiveForceScale = 1;
    	
    	double repulsiveForceScale = 1;

    	
        PioneerRobot robot = new PioneerRobot();
        MedianFilter filter = new MedianFilter(robot);
        
        robot.setVerbose(true);
        robot.connect("localhost", Integer.parseInt(args[0]));
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
        robot.setVerbose(false); 

   //LOOP
        
        while (true) 
        {
        	
            double[] sensorReadings = filter.getFilteredSonarValues();
 
            TwoDVector attractiveVector = calcFAtt(robot, attractiveForceScale);
            TwoDVector repulsiveVector = calcFRep(sensorReadings, sonarAngles, repulsiveForceScale);
        	
         //   TwoDVector targetVector = TwoDVector.vectorAdd(attractiveVector, repulsiveVector);
            TwoDVector gradient = attractiveVector;	             
            Example.moveAlongGradient(robot, gradient);
                   
            
            System.out.println("------------------- ROBOT X " + robot.getXPos());
        	System.out.println("------------------- ROBOT Y " + robot.getYPos());
        }
        
        }
    


public static TwoDVector calcFAtt(PioneerRobot robot, double attractiveForceScale){
	TwoDVector FAtt = Example.getGoalBotCoords(robot);
	TwoDVector scaledFAtt = TwoDVector.scalarMult(FAtt, attractiveForceScale);
	return scaledFAtt;
	
	
}
    
public static TwoDVector calcFRep(double[] sonarReadings, short[] sonarAngles, double repulsiveForceScale){
	TwoDVector FRep = new TwoDVector(0,0);

	//Goes to the 7th sonar - i.e. iterates only through the front sonars.
	for(int i = 0; i < 7; i++){
		double sonarDist = sonarReadings[i];
		TwoDVector sonarVector = new TwoDVector(sonarDist * Math.cos(sonarAngles[i]), sonarDist * Math.sin(sonarAngles[i]));
		TwoDVector sonarUnitVector = TwoDVector.normalize(sonarVector);
		TwoDVector repulsiveVector = TwoDVector.scalarMult( sonarUnitVector, (repulsiveForceScale / (TwoDVector.magnitude(sonarVector) * TwoDVector.magnitude(sonarVector)))); 
		FRep = TwoDVector.vectorAdd(FRep, repulsiveVector);
		
	}
	
	return FRep;}


public static TwoDVector getGoalBotCoords(PioneerRobot robot){
	TwoDVector goalWorld = new TwoDVector(GOAL_X, GOAL_Y);
	TwoDVector currPosition = new TwoDVector(robot.getXPos(), robot.getYPos());
	TwoDVector goalBotCoords = TwoDVector.rotate(-robot.getOrientation(), TwoDVector.vectorSubtract(goalWorld, currPosition));
	
	return goalBotCoords;
	
}


public static double getHeadingFromVector(PioneerRobot robot, TwoDVector vector) {   	
	double vecX = vector.getX();// - robot.getXPos();
	double vecY = vector.getY();// - robot.getYPos();
	
	return Math.atan(vecY / vecX);
}

public static void moveAlongGradient(PioneerRobot robot, TwoDVector gradient){
	long forwardSpeed = Math.round(gradient.getX()); 
	long sidewaysComponent = Math.round(gradient.getY());
	
	byte wheelScale = (byte)Math.round(TOP_SPEED/Math.abs(Math.max(robot.getLeftWheelVelocity(), robot.getRightWheelVelocity())));
	byte iSpeed = (byte) forwardSpeed;
	byte jSpeed = (byte) (forwardSpeed + sidewaysComponent);
	
	if ((robot.getLeftWheelVelocity() > 10) || (robot.getRightWheelVelocity() > 10) ){
		iSpeed *= wheelScale;
		jSpeed *= wheelScale;
	}
	
	robot.vel2(iSpeed,jSpeed);
	
	System.out.println(gradient);
	
	//Thread.sleep((long)1000);
	
}

public static double getMagnitudeDoubles(double x, double y){
	
	return Math.sqrt((x*x)+(y*y));
	
	
}
    }

