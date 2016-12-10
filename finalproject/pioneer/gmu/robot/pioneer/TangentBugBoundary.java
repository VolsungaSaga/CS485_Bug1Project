package gmu.robot.pioneer;

public class TangentBugBoundary {

	private double xCoord; //Where is this boundary, approximately?
	private double yCoord;
	
	private int sensorIndex; //Which sensor in the list provided by the filter is it?
	public TangentBugBoundary(double distance, double angleDeg, int i) {
		// Given a sensor reading and the angle of the sensor, calculate the
		// approximate
		// x and y coord of the boundary point.
		double angleRad = Math.toRadians(angleDeg);
		this.setxCoord(distance * Math.cos(angleRad));
		this.setyCoord(distance * Math.sin(angleRad));
		this.setSensorIndex(i);

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
	
	//Distance of Boundary
	
	public double getDistance(){
		return Math.sqrt(((this.xCoord * this.xCoord) + (this.yCoord * this.yCoord)));
		
	}

	public String toString(){
		String printString = String.format("\nI'm number %d! : {X: %.2f, Y: %.2f}\n", this.sensorIndex, this.xCoord, this.yCoord);
		return printString;
	}
}
