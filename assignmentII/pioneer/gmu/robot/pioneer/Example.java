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
	public static final short TOP_SPEED = 50;
	
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
        
        while ((robot.getXPos() != GOAL_X || robot.getYPos() != GOAL_Y) || 
        		((Math.abs(GOAL_X - robot.getXPos()) < 50 ) ||
        		((Math.abs(GOAL_Y - robot.getYPos())) < 50))) 
        {
        	
            double[] sensorReadings = filter.getFilteredSonarValues();
 
            TwoDVector attractiveVector = calcFAtt(robot, attractiveForceScale);
            TwoDVector repulsiveVector = calcFRep(sensorReadings, sonarAngles, repulsiveForceScale);
        	
            TwoDVector targetVector = TwoDVector.vectorAdd(attractiveVector, repulsiveVector);
            
            double targetHeading = Example.getHeadingFromCurrPosition(robot, targetVector);
            
            moveAlongHeading(robot, targetHeading, targetVector);
                   
            
            System.out.println("------------------- ROBOT X " + robot.getXPos());
        	System.out.println("------------------- ROBOT Y " + robot.getYPos());
        }
        

        

        robot.e_stop(); 
        robot.enable(false); 
        System.out.println(robot.getXPos()); 
        try { 
            Thread.sleep(2000); } catch (Exception e) {} 


        robot.disconnect();
        }
    


public static TwoDVector calcFAtt(PioneerRobot robot, double attractiveForceScale){
	TwoDVector FAtt = new TwoDVector(GOAL_X - robot.getXPos(),GOAL_Y - robot.getYPos());
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



public static double getHeadingFromCurrPosition(PioneerRobot robot, TwoDVector vector) {   	
	double vecX = vector.getX() - robot.getXPos();
	double vecY = vector.getY() - robot.getYPos();
	
	return Math.atan(vecY / vecX);
}

public static void moveAlongHeading(PioneerRobot robot, double heading, TwoDVector targetVector){
	short roundedMagnitude = (short)Math.round(TwoDVector.magnitude(targetVector)); 
	if(roundedMagnitude > TOP_SPEED){
		roundedMagnitude = TOP_SPEED;
	}
	
	short headingDegrees = (short)Math.round(Math.toDegrees(heading));
	System.out.printf("Heading Degrees: %d\n", headingDegrees);
	System.out.printf("Speed: %d\n", roundedMagnitude);
	System.out.println(targetVector);
	if((5 < headingDegrees) && (headingDegrees <= 180)){
		robot.rvel((short)5);
		
	}
	
	else if((180 < headingDegrees) && (headingDegrees < 354)){
		robot.rvel((short)-5);
	}
	
	else{
		robot.rvel((short)0);
	}
	//Thread.sleep((long)1000);
	
}
    }

