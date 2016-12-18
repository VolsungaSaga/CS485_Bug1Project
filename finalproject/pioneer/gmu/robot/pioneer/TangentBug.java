/*
 * Example.java
 */



package gmu.robot.pioneer;
import java.util.*;

/**
 * 
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
	public static double GOAL_X = 1000;
	public static double GOAL_Y = 0;
	public static ArrayList<ArrayList<TangentBugBoundary>> obstacleBounds;

	public static enum OuterStates {START, GO_TO_GOAL_STATE, BOUNDARY_FOLLOW_STATE, WALL_FOLLOW_STATE };
	public static enum WallFollowStates {TURNING_PARALLEL_STATE, STRAIGHT_WITH_TUNING_STATE, LOST_VISUAL_STATE};

	//Initialization of States and stuff.
	public static OuterStates OuterState = OuterStates.START;
	public static WallFollowStates WallFollowState = WallFollowStates.TURNING_PARALLEL_STATE; 
	
	//Flags and Constants for WallFollowState - might add some more later.
	
	//If it's true, obstacle's to the left. If it's false, obstacle's to the right.
	public static boolean obstDirection = true;
	public static final double WALL_FOLLOW_RANGE = 600;
	public static TangentBugBoundary lastHeuristicBound = null;
	//Sensory Stuff
	/*		SonarNum 0   1   2   3   4    5    6    7    8     9     10    11   12   13  14   15*/
	public static final short[] sonarAngles = {90, 50, 30, 10, -10, -30, -50, -90, -90, -130, -150, -170, 170, 150, 130, 90};

	public static void usage() throws Exception
	{
		System.out.println("Usage: Example <port>");
		String[] derp = {"5000"};
		main(derp);
	}

	public static final void main( String[] args ) throws Exception
	{
		//Initial Bullcrap...
		if (args.length<1)
			usage();

		PioneerRobot robot = new PioneerRobot();    
		MedianFilter filter = new MedianFilter(robot);

		robot.connect("localhost", Integer.parseInt(args[0]));
		robot.sonar( true );
		robot.enable( true );
		robot.setVerbose(false); 



		boolean check = true;
		try { 
			Thread.sleep(5000);
		} catch (Exception e) {}

		//THE MAIN 
		while(check){
			//The calculation of the Bounds of Obstacles. 

			obstacleBounds = calculateObstacleBounds(filter, robot);	 
			double [] sensorReadings = filter.getFilteredSonarValues();
			switch (OuterState) {
			case START:
				GoToGoToGoal_ST(obstacleBounds, robot);
				break;
			case GO_TO_GOAL_STATE: 
				//Do stuff!
				if(!IsGoalUnoccluded(obstacleBounds, robot)){
					GoToBoundaryFollow_ST(obstacleBounds, robot);
				}

				break;

			case BOUNDARY_FOLLOW_STATE: 
				//Do stuff!
				if(somethingInFront(sensorReadings)){
					
					GoToWallFollow_ST();
				}
				
				else if(!IsGoalUnoccluded(obstacleBounds, robot)){
					System.out.println("In Boundary Follow State, goal still occluded!");
					GoToBoundaryFollow_ST(obstacleBounds, robot);
				}

				break;
			case WALL_FOLLOW_STATE:
				//Do stuff!
				switch(WallFollowState){

				case TURNING_PARALLEL_STATE:
					if(!somethingInFront(sensorReadings)){
						GoToGoStraight_SubST(robot);
					}
					break;
				case STRAIGHT_WITH_TUNING_STATE:
					if(!somethingToSides(sensorReadings, obstDirection)){
						GoToLostVisual_SubST(robot);
					}
					break;
				case LOST_VISUAL_STATE:
					if(IsGoalUnoccluded(obstacleBounds, robot) ){
						GoToGoToGoal_ST(obstacleBounds, robot);
					}
					
					else if(somethingToSides(sensorReadings, obstDirection)){
						GoToGoStraight_SubST(robot);
					}
					
					break;

				}
				break;
			}

		}

		robot.e_stop();
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

	//STATE TRANSITION FUNCTIONS

	public static void GoToGoToGoal_ST(ArrayList<ArrayList<TangentBugBoundary>> boundLists, PioneerRobot robot){
		OuterState = OuterStates.GO_TO_GOAL_STATE;
		WallFollowState = WallFollowStates.TURNING_PARALLEL_STATE;
		//Do some motion command - go to point generic function, probably.	
		double angleToGoalRads = Math.atan((robot.getYPos() - GOAL_Y)/(robot.getXPos() - GOAL_X)); 
		doRotate(robot, angleToGoalRads, GOAL_X, GOAL_Y);
	} //Reached from Wall Follow State or Start

	public static boolean IsGoalUnoccluded(ArrayList<ArrayList<TangentBugBoundary>> boundLists, PioneerRobot robot){
		//printSomething();
		for(ArrayList<TangentBugBoundary> boundList : boundLists){
			//If the list we're looking at is of size 0, then don't bother with it.
			if(boundList.size() != 0){
				TangentBugBoundary firstBound = boundList.get(0);
				TangentBugBoundary lastBound = boundList.get(boundList.size()-1);
				double angleToFirst = firstBound.getAngleRadToBound(robot);
				//System.out.printf("\n\nAngle to First Bound: %.2f\n", angleToFirst);
				double angleToLast = lastBound.getAngleRadToBound(robot);
				//System.out.printf("Angle to Last Bound: %.2f\n", angleToLast);

				double angleToGoal = Math.atan((robot.getYPos() - GOAL_Y)/(robot.getXPos() - GOAL_X));
				//System.out.printf("Angle to Goal: %.2f\n", angleToGoal);

				if((angleToFirst <= angleToGoal )&&(angleToGoal <= angleToLast)){
					return false;
				}	
			}

		}
		return true;
	}

	//This transition does a couple of things - first, it gets a bound that satisfies the heuristic equation. Then, it stores that bound in a global variable for later.
	// Then, it flips the obstacle switch according to whether the robot went to a bound on its right or its left.
	//Reached from Go To Goal State or Boundary Follow
	public static void GoToBoundaryFollow_ST(ArrayList<ArrayList<TangentBugBoundary>> boundLists, PioneerRobot robot){
		OuterState = OuterStates.BOUNDARY_FOLLOW_STATE; 
		TangentBugBoundary heuristicBound = getHeuristicBound(boundLists, robot);
		System.out.println(heuristicBound.toString());
		//lastHeuristicBound = heuristicBound;
		//Before we actually go anywhere, let's set obstacle direction by accessing the sensor index.
		if((heuristicBound.getSensorIndex() <= 3 || heuristicBound.getSensorIndex() >= 12 )){
			obstDirection = true; //If it's sensors 0-3 or 12-15, it's to our left.
			
		}
		
		else{ obstDirection = false;} //If not, it's to our right.
		
		GoToHeuristicBound_SubST(heuristicBound, robot);
	} 

	//Tells the robot to go to a boundary point, using doRotate().
	public static void GoToHeuristicBound_SubST(TangentBugBoundary heuristicBound, PioneerRobot robot){
		if(heuristicBound == null){
			//System.out.printf("***Null heuristic bound!***");
			return;
		}

		else{
			System.out.printf("Going to Boundary: %.2f , %.2f", heuristicBound.getxCoord(), heuristicBound.getyCoord());
			doRotate(robot, heuristicBound.getAngleRadToBound(robot), heuristicBound.getxCoord(), heuristicBound.getyCoord());
		}

	} //Meant to be an internal function for BoundaryFollow_ST

	//Returns the boundary point that minimizes this heuristic equation: dist_RobotToBound + dist_BoundToGoal
	public static TangentBugBoundary getHeuristicBound(ArrayList<ArrayList<TangentBugBoundary>> boundLists, PioneerRobot robot){
		TangentBugBoundary heuristicBound = null;
		for(ArrayList<TangentBugBoundary> boundList: boundLists){
			if(boundList.size() != 0){
				TangentBugBoundary firstBound = boundList.get(0);
				TangentBugBoundary lastBound = boundList.get(boundList.size()-1);
				if((heuristicBound == null) || (firstBound.getHeuristicDistance(robot, GOAL_X, GOAL_Y) < heuristicBound.getHeuristicDistance(robot, GOAL_X, GOAL_Y)) ){
					heuristicBound = firstBound;

				}

				if(((heuristicBound == null) || (lastBound.getHeuristicDistance(robot, GOAL_X, GOAL_Y) < heuristicBound.getHeuristicDistance(robot, GOAL_X, GOAL_Y)))){
					heuristicBound = lastBound;

				}
			}


		}

		return heuristicBound;
	} //Gets the bound that satisfies the heuristic formula for TangentBug.
	//Reached from Boundary Follow.
	public static void GoToWallFollow_ST(){
		OuterState = OuterStates.WALL_FOLLOW_STATE;
		WallFollowState = WallFollowStates.TURNING_PARALLEL_STATE;
	} 

	public static void GoToTurningParallel_SubST(PioneerRobot robot){
		WallFollowState = WallFollowStates.TURNING_PARALLEL_STATE;
			if(obstDirection == false){
				robot.vel2((byte)-1,(byte) 1);
			}
			
			else{
				robot.vel2((byte) 1, (byte) -1);
				
			}
		
		
		
	}
	
	public static void GoToGoStraight_SubST(PioneerRobot robot){
		WallFollowState = WallFollowStates.STRAIGHT_WITH_TUNING_STATE;
		robot.vel2((byte) 1, (byte) 1);
	}
	
	public static void GoToLostVisual_SubST(PioneerRobot robot){
		WallFollowState = WallFollowStates.LOST_VISUAL_STATE;
		robot.stop(); //Stop!
		//If the obstacle was to the right, we must rotate right.
		if(obstDirection == false){
			for(int i = 0; i < 100; i++){
				robot.dhead((short)-90); //Hammer time!
			}
		}
		//If the obstacle was to the left, we must rotate left.
		else{
			for(int i = 0; i< 100; i++){
				robot.dhead((short)90);
			}
			
		}
		
		//Then, move forward.
		robot.vel2((byte) 1, (byte) 1);
		
	}

	public static boolean somethingInFront(double[] sensorReadings){
		return (sensorReadings[3] < WALL_FOLLOW_RANGE || sensorReadings[4] < WALL_FOLLOW_RANGE);
		
	}
	
	public static boolean somethingToSides(double[] sensorReadings, boolean obstacleDirection){
		boolean obstRightExpression = (sensorReadings[7] < WALL_FOLLOW_RANGE || sensorReadings[8] < WALL_FOLLOW_RANGE);
		boolean obstLeftExpression = (sensorReadings[0] < WALL_FOLLOW_RANGE || sensorReadings[15] < WALL_FOLLOW_RANGE);
		//The boolean argument acts as a switch - if you pass in a true (left) value, then it will return the status of the left sensors.
		//Vice versa for a false (right) value.
		return ((obstacleDirection == false) && obstRightExpression) || ((obstacleDirection == true) && obstLeftExpression);
		
	}
	



	//Returns a list of boundary lists, representing each of the currently detected
	// obstacles

	public static  ArrayList<ArrayList<TangentBugBoundary>> calculateObstacleBounds(MedianFilter filter, PioneerRobot robot) {
		ArrayList<ArrayList<TangentBugBoundary>> boundLists = new ArrayList<ArrayList<TangentBugBoundary>>();
		ArrayList<TangentBugBoundary> currentBoundList = new ArrayList<TangentBugBoundary>();
		//There will always be at least one group of bounds, though it could be empty (representing no obstacles). 
		boundLists.add(currentBoundList);
		double[] sensorReadings = filter.getFilteredSonarValues();

		//If a given sensor's previous neighbor doesn't see anything, but the given sensor does, then the given sensor
		// shall represent an obstacle bound (the point at which the LOS of the sensor is tangent to the edge of what it thinks is the obstacle).
		for(int i = 0; i < sensorReadings.length; i++){

			int previousNeighbor = TangentBugBoundary.getPreviousNeighbor(i);
			int nextNeighbor = TangentBugBoundary.getNextNeighbor(i);
			//Is this sensor in range, but one of its neighbors isn't?
			if(((sensorReadings[i] < SENSOR_RANGE) && !(sensorReadings[previousNeighbor] < SENSOR_RANGE)) ){
				ArrayList<TangentBugBoundary> newBoundList = new ArrayList<TangentBugBoundary>();
				boundLists.add(newBoundList);
				currentBoundList = newBoundList;
				currentBoundList.add(new TangentBugBoundary(sensorReadings[i], sonarAngles[i], i, robot));

			}

			//Is this sensor in range, along with its neighbors?
			else if ((sensorReadings[i] < SENSOR_RANGE)  ||
					((sensorReadings[i] < SENSOR_RANGE) && !(sensorReadings[nextNeighbor] < SENSOR_RANGE))){
				currentBoundList.add(new TangentBugBoundary(sensorReadings[i], sonarAngles[i], i, robot));

			}	
			//If it isn't in range, don't put it in the obstacle group.
			else{}
		}

		return boundLists;

	}


	public static void printSomething() { 
		for(int i = 0; i < obstacleBounds.size(); i++){
			System.out.printf("\n---- OBSTACLE GROUP %d ----\n", i);
			for(int j = 0; j < obstacleBounds.get(i).size(); j++){
				System.out.printf("%s", obstacleBounds.get(i).get(j).toString());

			}
		}
	}

	public static void doRotate(PioneerRobot robot, double angleRads, double globalX, double globalY){
		//rotate and go to the goal
		double angleDeg = Math.toDegrees(angleRads);
		System.out.printf("angleRad: %.2f", angleRads);
		System.out.printf("angleDeg: %.2f", angleDeg);
		// first or second quadrant.
		if ((globalX >= 0 && globalY >= 0)) {
			System.out.println("First quad");
			for(int i=0; i<100;i++){
				robot.head((short) angleDeg);
			}

		} else if ((globalX < 0 && globalY > 0)) {
			System.out.println("second quad");

			for(int i=0; i<100;i++){
				robot.head((short) (180 + angleDeg));
			}
		} else if (globalX < 0 && globalY < 0) {
			System.out.println("third quad");

			for(int i=0; i<100;i++){
				robot.head((short) (180 - angleDeg));
			}
		} else if (globalX > 0 && globalY < 0) {
			System.out.println("fourth quad");

			for(int i=0; i<100;i++){
				robot.head((short) (360 + angleDeg));
			}
		}

		robot.vel2((byte) 2, (byte) 2);
	}
}
