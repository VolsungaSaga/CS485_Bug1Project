package gmu.robot.pioneer;

public class TangentBugBoundary {

	private double xCoord; //Where is this boundary, approximately?
	private double yCoord;
	
	private int sensorIndex; //Which sensor in the list provided by the filter is it?
	public TangentBugBoundary(double distance, double angleDeg, int i) {
		// Given a sensor reading and the angle of the sensor, calculate the
		// approximate
		// x and y coord of the boundary point.

		this.xCoord = distance * Math.cos(angleDeg);
		this.yCoord = distance * Math.sin(angleDeg);
		this.sensorIndex = i;

	}

}
