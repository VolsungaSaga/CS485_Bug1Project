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

  
 static double goalX = -200;
 static double goalY = 700;
 
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
  
  int state = 1;
  boolean check = true;

  try {
   Thread.sleep(5000);
  } catch (Exception e) {
  }

  //MedianFilter filter = new MedianFilter(robot);

  while (check) {

   switch (state) {
   case 0: // reached the goal state
    check = false;
    break;
   case 1: // move all toward the goa
    double angleRads = Math.atan(Math.abs(goalY - robot.getYPos()) / 
       Math.abs(goalX - robot.getXPos()));
    
    System.out.println("subtraction " + Math.abs(robot.getOrientation() - angleRads));
    System.out.println("angle calc " + angleRads);
    System.out.println("Robot Or " + robot.getOrientation());

     doRotate(robot, angleRads);
     

     robot.vel2((byte) 5, (byte) 5);
    
    if (Math.abs(robot.getXPos()) > Math.abs(goalX) && Math.abs(robot.getYPos()) > Math.abs(goalY)) {
       System.out.println(robot.getXPos());
       System.out.println(robot.getYPos());
      state = 0;
    }
   }

  }

  robot.e_stop();
  
  try {
   Thread.sleep(2000);
  } catch (Exception e) {
  }

  /*
   * for( short i = 0 ; i < 10 ; i++ ) { robot.sound(i); robot.dhead(
   * (short)(60 * (short)(2*(i%2)-1)) ); Thread.sleep(1000); }
   */
  robot.disconnect();
 }
 
 public static void doRotate(PioneerRobot robot, double angleRads){
   //rotate and go to the goal
        double angleDeg = Math.toDegrees(angleRads);
     // first or second quadrant.
     if ((goalX >= 0 && goalY >= 0)) {
      System.out.println("First quad");
      for(int i=0; i<100;i++){
        robot.head((short) angleDeg);
      }
      
     } else if ((goalX < 0 && goalY > 0)) {
      System.out.println("second quad");
      
      for(int i=0; i<100;i++){
        robot.head((short) (180 - angleDeg));
      }
     } else if (goalX < 0 && goalY < 0) {
      System.out.println("third quad");
     
      for(int i=0; i<100;i++){
         robot.head((short) (180 + angleDeg));
      }
     } else if (goalX > 0 && goalY < 0) {
      System.out.println("forth quad");
     
      for(int i=0; i<100;i++){
        robot.head((short) (360 - angleDeg));
      }
     }
 }
}
