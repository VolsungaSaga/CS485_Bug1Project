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
	
	//According to the Pioneer 3's operator manual, these are the angles at 
	// which sonars 0 - 7 are located. I don't know if these mesh with the Pioneer's 
	// internal understanding of its own orientation, but nothing ventured, nothing gained.
	
	/*		SonarNum:		0    1    2    3    4  5   6   7 */
	short[] sonarAngles = {270, 310, 330, 350, 10, 30, 50, 90};

	double repulsiveForceScale = 1;
	
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
        
        //robot.vel2((byte)10, (byte)10); 
//        try { 
//            Thread.sleep(5000);
//            } catch (Exception e) {}

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
    


public TwoDVector calcFRep(double[] sonarReadings){
	TwoDVector FRep = new TwoDVector(0,0);

	//Goes to the 7th sonar - i.e. iterates only through the front sonars.
	for(int i = 0; i < 8; i++){
		double sonarDist = sonarReadings[i];
		TwoDVector sonarVector = new TwoDVector(sonarDist * Math.cos(sonarAngles[i]), sonarDist * Math.sin(sonarAngles[i]));
		TwoDVector sonarUnitVector = TwoDVector.normalize(sonarVector);
		TwoDVector repulsiveVector = TwoDVector.scalarMult( sonarUnitVector, (repulsiveForceScale / (TwoDVector.magnitude(sonarVector) * TwoDVector.magnitude(sonarVector)))); 
		FRep = TwoDVector.vectorAdd(FRep, repulsiveVector);
		
	}
	
	return FRep;}

    }


