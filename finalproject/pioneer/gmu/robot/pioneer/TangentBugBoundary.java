package gmu.robot.pioneer;

public class TangentBugBoundary {

	private double xCoord; //Where is this boundary, approximately?
	private double yCoord;
	private boolean isLeft;
	
	private int sensorIndex; //Which sensor in the list provided by the filter is it?
	public TangentBugBoundary(double distance, double angleDeg, int i) {
		// Given a sensor reading and the angle of the sensor, calculate the
		// approximate
		// x and y coord of the boundary point.
		double angleRad = Math.toRadians(angleDeg);
		this.setxCoord(distance * Math.cos(angleRad));
		this.setyCoord(distance * Math.sin(angleRad));
		this.setSensorIndex(i);
		this.isLeft = false;
	}
	
	public void setLeft(boolean isLeft) {
		this.isLeft = isLeft;
	}
	
	public boolean getLeft() {
		return isLeft;
	}
	
	//Get ___ Neighbor: These functions return the index of the previous or next sensor in the sonar array that we've got. 
	public static int getPreviousNeighbor(int i){
		if(i == 0)
			return 15; //The numbering goes from 0 - 15, so we want to cycle back to 15.
		return i-1;
	}
	
	public static int getNextNeighbor(int i){
		if(i == 15)
			return 0; //As in the previous-neighbor function, we want to cycle, this time to sonar 0.
		return i+1;
	}
	
	
	//Auto-genn'd getters and setters! Woo!
	
	//X Coordinate
	public double getxCoord() {
		return xCoord;
	}
	public void setxCoord(double xCoord) {
		this.xCoord = xCoord;
	}
	
	//Y-Coordinate
	public double getyCoord() {
		return yCoord;
	}
	public void setyCoord(double yCoord) {
		this.yCoord = yCoord;
	}
	
	//Sensor Index
	public int getSensorIndex() {
		return sensorIndex;
	}
	public void setSensorIndex(int sensorIndex) {
		this.sensorIndex = sensorIndex;
	}
	
	//Helper Methods for calculating heuristic distance.
	
	public double getRobotDistanceToBoundary(PioneerRobot robot){
		return Math.sqrt((Math.pow((robot.getXPos() - this.xCoord), 2) + Math.pow((robot.getYPos() - this.yCoord), 2)));
		
	}
	
	public double getBoundDistanceToGoal(double goalX, double goalY){
		return Math.sqrt((Math.pow((goalX - this.xCoord), 2) + Math.pow((goalY - this.yCoord), 2)));

	}
	
	public double getHeuristicDistance(PioneerRobot robot, double goalX, double goalY){
		return this.getRobotDistanceToBoundary(robot) + this.getBoundDistanceToGoal(goalX, goalY);
		
	}
	
	//Angle to Boundary - From the CURRENT position of the robot.
	public double getAngleRadToBound(PioneerRobot robot) {
		double radianAngle = Math.atan((robot.getYPos() - this.yCoord)/(robot.getXPos() - this.xCoord));
		return radianAngle;
	}

	public String toString(){
		String printString = String.format("\nI'm number %d! : {X: %.2f, Y: %.2f}\n", this.sensorIndex, this.xCoord, this.yCoord);
		return printString;
	}
}
