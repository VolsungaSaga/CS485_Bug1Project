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
				double angleRads = Math.atan(Math.abs(goalY - robot.getYPos()) / 
						 Math.abs(goalX - robot.getXPos()));
				
				System.out.println("subtraction " + Math.abs(robot.getOrientation() - angleRads));
				System.out.println("angle calc " + angleRads);
				System.out.println("Robot Or " + robot.getOrientation());

		//		if (Math.abs(robot.getOrientation() - angleRads) > 0.01) {
					
					double angleDeg = Math.toDegrees(angleRads);
					// first or second quadrant.
					if ((goalX >= 0 && goalY >= 0)) {
						System.out.println("First quad");
						robot.dhead((short) angleDeg);
					} else if ((goalX < 0 && goalY > 0)) {
						System.out.println("second quad");
						robot.dhead((short) (180 - angleDeg));
					} else if (goalX < 0 && goalY < 0) {
						System.out.println("third quad");
						robot.dhead((short) (180 + angleDeg));
					} else if (goalX > 0 && goalY < 0) {
						System.out.println("forth quad");
						robot.dhead((short) (360 - angleDeg));
					}
					// Thread.sleep(1000);
		//		} else {
					while (!robot.isValidData())
					{
						System.out.println("not valid");
					}
					robot.vel2((byte) 5, (byte) 5);
	//			}
				
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
