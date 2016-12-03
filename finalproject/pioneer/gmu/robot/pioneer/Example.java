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

	public static void usage() throws Exception {
		System.out.println("Usage: Example <port>");
		// System.exit(0);
		String[] derp = { "5000" };
		main(derp);
	}

	public static final void main(String[] args) throws Exception {
		if (args.length < 1)
			usage();

		PioneerRobot robot = new PioneerRobot();
		robot.setVerbose(true);
		robot.connect("127.0.0.1", Integer.parseInt(args[0]));
		robot.sonar(true);
		robot.enable(true);
		robot.setVerbose(true);
		double goalX = 1000;
		double goalY = 1000;
		int state = 1;
		boolean check = true;

		try {
			Thread.sleep(5000);
		} catch (Exception e) {
		}

		MedianFilter filter = new MedianFilter(robot);

		while (check) {

			switch (state) {
			case 0: // reached the goal state
				check = false;
				break;
			case 1: // move all toward the goal
				double angleDeg = Math
						.toDegrees(Math.atan(Math.abs(goalY - robot.getYPos()) / Math.abs(goalX - robot.getXPos())));
				
				if (Math.abs(robot.getOrientation() - angleDeg) > 0.1) {
					// first or second quadrant.
					if ((goalX >= 0 && goalY >= 0)) {
						robot.dhead((short) angleDeg);
					} else if ((goalX < 0 && goalY > 0)) {
						robot.dhead((short) (180 - angleDeg));
					} else if (goalX < 0 && goalY < 0) {
						robot.dhead((short) (180 + angleDeg));
					} else if (goalX > 0 && goalY < 0) {
						robot.dhead((short) (360 - angleDeg));
					}
				} else {
					robot.vel2((byte) 5, (byte) 5);
				}
				
				if (robot.getXPos() > goalX && robot.getYPos() > goalY) {
					state = 0;
				}
			}

		}
		robot.e_stop();

		try {
			Thread.sleep(2000);
		} catch (Exception e) {
		}
		System.out.println(robot.getXPos());

		/*
		 * for( short i = 0 ; i < 10 ; i++ ) { robot.sound(i); robot.dhead(
		 * (short)(60 * (short)(2*(i%2)-1)) ); Thread.sleep(1000); }
		 */
		robot.disconnect();
	}
}
