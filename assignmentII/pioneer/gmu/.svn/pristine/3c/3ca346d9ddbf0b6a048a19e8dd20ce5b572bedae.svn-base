/*
 * Example.java
 */

package gmu.robot.pioneer;

public class AvoidObstaclesOld
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
        
    public static void main( String[] args ) throws Exception
        {        
        if (args.length<1)
            usage();

        double[] angles = {-90,-50,-30,-10,10,30,50,90,90,130,150,170,-170,-150,-130,-90};
        for(int x=0;x<angles.length;x++)
            angles[x] = (angles[x] / 180) * Math.PI;

        PioneerRobot robot = new PioneerRobot();
        robot.setVerbose(true);
        robot.connect("localhost", Integer.parseInt(args[0]));
        robot.sonar( true );
        robot.enable( true );
        double fx = 0;
        double fy = 0;
        double strength = 0;

        while(true)
            {
            // gather angle
            // Equation is: += 1/Sqrt(dist) * unit in direction
            double x = 0;
            double y = 0;
            double mag = 100000;
            double[] sonars = robot.getSonars();
            int min = (sonars.length < 16 ? sonars.length: 16);
            for(int i = 0 ; i < min && sonars[i] == sonars[i]; i++)
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
            Thread.sleep(10);
            }
        }
    }