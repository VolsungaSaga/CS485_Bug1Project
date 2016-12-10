/*
 * Example.java
 */



package gmu.robot.pioneer;
import java.util.*;

/**
 * This is the sample program for controlling an AmigoBot robot.
 * <P>
 * <A HREF="../COPYRIGHT.html">Copyright</A>
 * (c)2001 Evolutinary Computation Laboratory @ George Mason University
 *
 * @author Liviu Panait
 * @version 1.0
 */

public class TangentBug
{

	public static final double SENSOR_RANGE = 1000;
	public static final double GOAL_X = 1000;
	public static final double GOAL_Y = 1000;


	//Sensory Stuff
	/*		SonarNum:							0   1   2   3   4    5    6    7    8     9     10    11   12   13  14   15*/
	public static final short[] sonarAngles = {90, 50, 30, 10, -10, -30, -50, -90, -90, -130, -150, -170, 170, 150, 130, 90};

	public static void usage() throws Exception
	{
		System.out.println("Usage: Example <port>");
		//        System.exit(0);
		String[] derp = {"5000"};
		main(derp);
	}

	public static final void main( String[] args ) throws Exception
	{
		if (args.length<1)
			usage();

		PioneerRobot robot = new PioneerRobot();    
		MedianFilter filter = new MedianFilter(robot);

		robot.setVerbose(true);
		robot.connect("localhost", Integer.parseInt(args[0]));
		robot.sonar( true );
		robot.enable( true );
		robot.setVerbose(true); 
		double goalX = 1000;
		double goalY = 1000;


		int state = 1;
		boolean check = true;
		//        robot.dhead(a);
		//        robot.move(b);

//		robot.vel2((byte)10, (byte)10); 
		try { 
			Thread.sleep(5000);
		} catch (Exception e) {}

		while(check){
			//The calculation of the Bounds of Obstacles. 

			ArrayList<ArrayList<TangentBugBoundary>> obstacleBounds = calculateObstacleBounds(filter);	 

			/*
			switch(state){
			case 0: //finished
				check = false;
				break;
			case 1: // move all toward the goal
				if(goalX >= 0 && goalY >=0){
					robot.dhead((short) Math.toDegrees(Math.atan((goalY-robot.getYPos())/(goalX-robot.getXPos()))));
				}
				robot.vel2((byte)5, (byte)5);
				if(robot.getXPos()>goalX && robot.getYPos()>goalY){
					state = 0;
				}*/
		
			//Test Code - Are the obstacle groups forming correctly?
			for(int i = 0; i < obstacleBounds.size(); i++){
				System.out.printf("\n---- OBSTACLE GROUP %d ----\n", i);
				for(int j = 0; j < obstacleBounds.get(i).size(); j++){
					System.out.printf("%s", obstacleBounds.get(i).get(j).toString());
					
				}
			}
		}

		
		robot.e_stop(); 
		robot.enable(true); 
		System.out.println(robot.getXPos()); 
		try { 
			Thread.sleep(2000); } catch (Exception e) {} 
		//robot.resetOdometery(); 
		System.out.println(robot.getXPos()); 

		/*for( short i = 0 ; i < 10 ; i++ )
	           {
	           robot.sound(i);
	           robot.dhead( (short)(60 * (short)(2*(i%2)-1)) );
	           Thread.sleep(1000);
	           }
		 */
		robot.disconnect();
	}

	//Returns a list of boundary lists, representing each of the currently detected
	// obstacles

	public static  ArrayList<ArrayList<TangentBugBoundary>> calculateObstacleBounds(MedianFilter filter) {
		ArrayList<ArrayList<TangentBugBoundary>> boundLists = new ArrayList<ArrayList<TangentBugBoundary>>();
		ArrayList<TangentBugBoundary> currentBoundList = new ArrayList<TangentBugBoundary>();
		//There will always be at least one group of bounds, though it could be empty (representing no obstacles). 
		boundLists.add(currentBoundList);
		double[] sensorReadings = filter.getFilteredSonarValues();

		//If a given sensor's previous neighbor or next neighbor doesn't see anything, but the given sensor does, then the given sensor
		// shall represent an obstacle bound (the point at which the LOS of the sensor is tangent to the edge of what it thinks is the obstacle).
		for(int i = 0; i < sensorReadings.length; i++){

			int previousNeighbor = TangentBugBoundary.getPreviousNeighbor(i);
			int nextNeighbor = TangentBugBoundary.getNextNeighbor(i);
			//Is this sensor in range, but one of its neighbors isn't?
			if(((sensorReadings[i] < SENSOR_RANGE) && !(sensorReadings[previousNeighbor] < SENSOR_RANGE)) ||
					((sensorReadings[i] < SENSOR_RANGE) && !(sensorReadings[nextNeighbor] < SENSOR_RANGE))){
				ArrayList<TangentBugBoundary> newBoundList = new ArrayList<TangentBugBoundary>();
				boundLists.add(newBoundList);
				currentBoundList = newBoundList;
				currentBoundList.add(new TangentBugBoundary(sensorReadings[i], sonarAngles[i], i));

			}

			//Is this sensor in range, along with its neighbors?
			else if (sensorReadings[i] < SENSOR_RANGE){
				currentBoundList.add(new TangentBugBoundary(sensorReadings[i], sonarAngles[i], i));

			}	
			//If it isn't in range, don't put it in the obstacle group.
			else{}
		}

		return boundLists;

	}



}
