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
	
	public static final int GOAL_X = 300;
	public static final int GOAL_Y = 300;
	public static final double SENSOR_RANGE = 1000;
	public static final short TOP_SPEED = 10;
	
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

