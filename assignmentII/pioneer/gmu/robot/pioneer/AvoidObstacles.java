/*
 * Example.java
 */

package gmu.robot.pioneer;

public class AvoidObstacles
    {
    
    static long every_time = 0;
    public static boolean every(long millis)
        {
        long cur = System.currentTimeMillis();
        if (every_time + millis < cur)
            {
            every_time = cur;
            return true;
            }
        else return false;
        }

    public static void usage()
        {
        System.out.println("Usage: AvoidObstacles <port>");
        System.exit(0);
        }
        
    public static double min(double a, double b, double c)
        {
        if (a < b && a < c) return a;
        if (b < c) return b; 
        return c;
        }
        
    public static boolean goAway(double[] sonars, int numSonars, PioneerRobot robot)
        {
        // gather angle
        // Equation is: += 1/Sqrt(dist) * unit in direction
        double fx = 0;
        double fy = 0;
        double strength = 0;
        double x = 0;
        double y = 0;
        double mag = 100000;

        for(int i = 0 ; i < numSonars; i++)
            {
            if (sonars[i] < mag)
                {
                mag = sonars[i];
                x = Math.cos(angles[i]);
                y = Math.sin(angles[i]);
                }
            }
                
        final double alpha = 0.5;
                
        if (mag > 2000) 
            {
            mag = 2000 ; // we don't care about bigger
            x = fx;
            y = fy;
            }
                        
        mag = 2000 - mag;
                
        strength = (1-alpha)*strength + alpha*mag;
        fx = (1-alpha)*fx + alpha*x ;
        fy = (1-alpha)*fy + alpha*y ;
                
        // other vector
        double tx = 0;
                
        // Now mix in some x
        final double FORWARD = 2;

        double heading = Math.atan2(fy,fx)*180/Math.PI;
        
        if (heading < 90 && heading > -90)  // back away
            {
            tx = -strength/100;
            tx = tx * (90-Math.abs(heading))/90;
            }
        else    // move forward
            {
            heading += 180;
            if (heading > 180) heading = -360 + heading;
            tx = strength/100;
            tx = tx * (90-Math.abs(heading))/90;
            }
        

        heading /= 5;  // cut-down
                
        robot.vel2((byte) (heading+tx),(byte)(-heading+tx));
        return true;
        }

    public static boolean goToOpenArea(double[] sonars, int numSonars, PioneerRobot robot)
        {
        // gather angle
        // Equation is: += 1/Sqrt(dist) * unit in direction
        double fx = 0;
        double fy = 0;
        double strength = 0;
        double x = 0;
        double y = 0;
        int i;
                
        double[] single = new double[16];
        double[] triple = new double[16];
        double[] quintuple = new double[16];
        double[] sestuple = new double[16];
                
        for(i = 0; i < numSonars; i++)
            {
            single[i] = sonars[i];
            triple[i] = min(single[i], sonars[(i+1)%numSonars], sonars[(i-1+numSonars)%numSonars]);
            quintuple[i] = min(triple[i], sonars[(i+2)%numSonars], sonars[(i-2+numSonars)%numSonars]);
            sestuple[i] = min(quintuple[i], sonars[(i+3)%numSonars], sonars[(i-3+numSonars)%numSonars]);
            }
                        
        // figure best as follows: I'm better than you if my sole sonar is better,
        // or if we're equal then if my triple neighborhood is better; else my quintuple neighborhood; else my sestuple neighborhood
        int best = 0;
        for(i = 0; i < numSonars; i++)
            {
            if (single[i] > single[best] ||
                single[i] == single[best] && triple[i] > triple[best] ||
                single[i] == single[best] && triple[i] == triple[best] && quintuple[i] > quintuple[best] ||
                single[i] == single[best] && triple[i] == triple[best] && quintuple[i] == quintuple[best] && sestuple[i] > sestuple[best])
                {
                best = i;
                } 
            }

        x = Math.cos(angles[best]);
        y = Math.sin(angles[best]);

        //if (single[best] < 100)  // everything is too close
        //      return false;

        // run through a filter
        final double alpha = 0.01;
        strength = (1-alpha)*strength + alpha*500;
        fx = (1-alpha)*fx + alpha*x ;
        fy = (1-alpha)*fy + alpha*y ;
        double heading = Math.atan2(fy,fx)*180/Math.PI;
        System.err.println(best);
                

        // R is the ICC
        // L is the width of the robot
        // omega is the rotational velocity
        // vl = omega (R - L/2)
        // vr = omega (R + L/2)
        // if the ICC is 0 we have
        // vl ~ -omega
        // vr ~ omega

        // make omega proportional to the difference
        double omega = heading;
        omega /= 30;
                
        double tx = 0;
        /*
          double tx = 0;
          if (heading < 35 && heading > -35)  // head for it
          {
          tx = 10;
          }
                        
          System.err.println("Heading + " + heading);
          if (tx > 0) System.err.println("Going for it");
        */

        robot.vel2((byte) (omega+tx),(byte)(-omega+tx));
        return true;
        }

    public static final double[] angles = {-90,-50,-30,-10,10,30,50,90,90,130,150,170,-170,-150,-130,-90};
        
    public static void main( String[] args ) throws Exception
        {        
        if (args.length<1)
            usage();

        for(int x=0;x<angles.length;x++)
            angles[x] = (angles[x] / 180) * Math.PI;

        PioneerRobot robot = new PioneerRobot();
        robot.setVerbose(true);
        robot.connect("localhost", Integer.parseInt(args[0]));
        robot.sonar( true );
        robot.enable( true );
        int lastDirection = 0;
        double scale = 1.5;

        while(true)
            {
            double[] sonars = robot.getSonars();
            int numSonars = (sonars.length < 16 ? sonars.length: 16);
            int sum = 0;
            double min = 10000;
            for(int i = 0; i < numSonars; i++) 
                { sum += sonars[i]; min = Math.min(min, sonars[i]); }
                                
            if (sum > 0)
                {
                if (min < 200) {System.err.println("Go AWAY"); goAway( sonars, numSonars, robot); }
                else if (sonars[0] < 200*scale || sonars[1] < 300*scale || sonars[2] < 500*scale || sonars[3] < 600*scale || sonars[4] < 600*scale || sonars[5] < 500*scale || sonars[6] < 300*scale || sonars[7] < 200*scale) // half a meter, turn 
                    {
                    if (lastDirection == 0)
                        lastDirection = (int)(Math.random() * 2) == 0 ? -1 : 1;
                    if (lastDirection == -1)
                        robot.vel2((byte)(10), (byte)(-10));
                    else
                        robot.vel2((byte)(-10), (byte)(10));
                    }
                else // forward
                    {
                    lastDirection = 0; 
                    robot.vel2((byte)(10), (byte)(10));
                    }
                }
            Thread.sleep(10);
            }
        }
    }