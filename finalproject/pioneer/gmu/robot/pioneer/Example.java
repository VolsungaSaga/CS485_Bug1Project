/*
 * Example.java
 */

package gmu.robot.pioneer;

/**
 * This is the sample program for controlling an AmigoBot robot.
 * <P>
 * <A HREF="../COPYRIGHT.html">Copyright</A> (c)2001 Evolutinary Computation
 * Laboratory @ George Mason University
 *
 * @author Liviu Panait
 * @version 1.0
 */

public class Example {

	public static final int GOAL_X = 300;
	public static final int GOAL_Y = 300;
	public static final double SENSOR_RANGE = 1000;
	public static final short TOP_SPEED = 10;

	public static void usage() {
		System.out.println("Usage: Example <port>");
		System.exit(0);
	}

	public static final void main(String[] args) throws Exception {
		if (args.length < 1)
			usage();

		// INITIALIZATION

		// According to the Pioneer 3's operator manual, these are the angles at
		// which sonars 0 - 7 are located. I don't know if these mesh with the
		// Pioneer's
		// internal understanding of its own orientation, but nothing ventured,
		// nothing gained.

		/* SonarNum: 0 1 2 3 4 5 6 7 */
		short[] sonarAngles = { 270, 310, 330, 350, 10, 30, 50, 90 };

		double attractiveForceScale = 1;

		double repulsiveForceScale = 1;

		PioneerRobot robot = new PioneerRobot();
		MedianFilter filter = new MedianFilter(robot);

		robot.setVerbose(true);
		robot.connect("localhost", Integer.parseInt(args[0]));
		robot.sonar(true);
		robot.enable(true);
		robot.setVerbose(false);

		// LOOP

		while (true) {

			double[] sensorReadings = filter.getFilteredSonarValues();

		}

	}

	public static double getMagnitudeDoubles(double x, double y) {

		return Math.sqrt((x * x) + (y * y));

	}
}
