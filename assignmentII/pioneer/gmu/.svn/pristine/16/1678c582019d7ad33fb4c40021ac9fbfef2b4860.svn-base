/*
 * PioneerPortrayal.java
 *
 * By Joey Harrison
 * Last Modified 12/8/07
 * 
 * This class draws an overhead view of a Pioneer3DX robot. It creates a polygon 
 * outline (in mm) and maintains a path history. When paint is called, it draws
 * both.
 * 
 * It implements the Portrayal2D class out of MASON to encourage reusable components
 * between MASON and this robot code.
 * 
 * Some potential improvements:
 *      - Allow the appearance to be customized 
 *                      (e.g. path history width, color, show points, etc.)
 *      - Generalize a framework so portrayals can be easily added for sensor 
 *              visualization, etc. Note: the path history should probably be an add-on.
 *
 */

package gmu.robot.pioneer.jgui.portrayals;

import java.awt.*;
import java.awt.geom.GeneralPath;
import java.awt.geom.Line2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
import java.util.ArrayList;

import sim.display.GUIState;
import sim.portrayal.DrawInfo2D;
import sim.portrayal.Inspector;
import sim.portrayal.LocationWrapper;

import gmu.robot.pioneer.jgui.Poseable2D;


public class PioneerPortrayal implements sim.portrayal.Portrayal2D, Poseable2D
    {
    private static final long serialVersionUID = 1L;
    private Polygon originalOutline;
    private Polygon outline;
        
    public double x = 0;
    public double y = 0;
    public double heading = 0;
        
    /*
     * This value can be used to draw a small vector in any direction you want.
     * For example, it can be used to show a commanded heading, or direction to
     * a beacon, etc. The flag showAltHeading must be set to true for this to appear.
     */
    public double altHeading = 0;
        
    /*
     * Set this to true if you want to draw an altHeading arrow.
     */
    public boolean showAltHeading = false;
        
    private Point altHeadingPt;
        
    private GeneralPath pathHistory;
    final static BasicStroke pathStroke = new BasicStroke(8.0f);
        
    public PioneerPortrayal()
        {
        originalOutline = new Polygon();

        originalOutline.addPoint(-223, -15);
        originalOutline.addPoint(-217, -56);
        originalOutline.addPoint(-204, -93);
        originalOutline.addPoint(-183, -129);
        originalOutline.addPoint(-159, -158);
        originalOutline.addPoint(-151, -167);
        originalOutline.addPoint(-139, -171);
        originalOutline.addPoint(-53, -171);
        originalOutline.addPoint(-46, -173);
        originalOutline.addPoint(-31, -188);
        originalOutline.addPoint(-22, -194);
        originalOutline.addPoint(104, -194);
        originalOutline.addPoint(117, -189);
        originalOutline.addPoint(131, -181);
        originalOutline.addPoint(154, -161);
        originalOutline.addPoint(172, -141);
        originalOutline.addPoint(186, -123);
        originalOutline.addPoint(200, -97);
        originalOutline.addPoint(211, -72);
        originalOutline.addPoint(218, -43);
        originalOutline.addPoint(222, -14);
        originalOutline.addPoint(222, 13);
        originalOutline.addPoint(218, 42);
        originalOutline.addPoint(211, 71);
        originalOutline.addPoint(200, 96);
        originalOutline.addPoint(186, 122);
        originalOutline.addPoint(172, 140);
        originalOutline.addPoint(154, 160);
        originalOutline.addPoint(131, 180);
        originalOutline.addPoint(117, 188);
        originalOutline.addPoint(104, 193);
        originalOutline.addPoint(-22, 193);
        originalOutline.addPoint(-31, 187);
        originalOutline.addPoint(-46, 172);
        originalOutline.addPoint(-53, 170);
        originalOutline.addPoint(-139, 170);
        originalOutline.addPoint(-151, 166);
        originalOutline.addPoint(-159, 157);
        originalOutline.addPoint(-183, 128);
        originalOutline.addPoint(-204, 92);
        originalOutline.addPoint(-217, 55);
        originalOutline.addPoint(-223, 14);
                
        outline = new Polygon();
                
        // copy the points to make room for future copying
        for (int i = 0; i < originalOutline.npoints; i++)
            outline.addPoint(originalOutline.xpoints[i], originalOutline.ypoints[i]);
                
        altHeadingPt = new Point(200, 0);
                
        pathHistory = new GeneralPath();
        pathHistory.moveTo(0, 0);
        }
        
    /*
     * Set the current pose of the robot. Point will be added to the path history.
     */
    public void setPose(double x, double y, double heading)
        {
        Line2D.Double line = new Line2D.Double(this.x, this.y, x, y);
        pathHistory.append(line, true);
                
        this.x = x;
        this.y = y;
        this.heading = heading;
        }
        
    public void clearPathHistory()
        {
        pathHistory.reset();
        }
        
    /*
     * Gather the path history into an ArrayList and return it.
     */
    public ArrayList<Point2D.Double> getPathHistoryPoints()
        {
        ArrayList<Point2D.Double> points = new ArrayList<Point2D.Double>();
        double data[] = new double[6];
                
        PathIterator iter = pathHistory.getPathIterator(null);
        while (!iter.isDone())
            {
            iter.currentSegment(data);
            points.add(new Point2D.Double(data[0], data[1]));
            iter.next();
            }
                
        return points;
        }

    private void rotate(Polygon poly, double theta)
        {
        final double sinTheta = Math.sin(theta);
        final double cosTheta = Math.cos(theta);
        int x, y;
        
        for (int i = 0; i < poly.npoints; i++)
            {
            x = poly.xpoints[i];
            y = poly.ypoints[i];
                
            poly.xpoints[i] = (int)(cosTheta * x - sinTheta * y);
            poly.ypoints[i] = (int)(sinTheta * x + cosTheta * y);
            }
        }

    private void rotate(Point p, double theta)
        {
        final double sinTheta = Math.sin(theta);
        final double cosTheta = Math.cos(theta);
        int x, y;
        
        x = p.x;
        y = p.y;
        
        p.x = (int)(cosTheta * x - sinTheta * y);
        p.y = (int)(sinTheta * x + cosTheta * y);
        }
        
    private void transformOutline()
        {
        // work from the original points so we don't accumulate error
        for (int i = 0; i < originalOutline.npoints; i++)
            {
            outline.xpoints[i] = originalOutline.xpoints[i];
            outline.ypoints[i] = originalOutline.ypoints[i];
            }
                
        // rotate to current heading
        rotate(outline, heading + Math.PI * 0.5);               // change heading so 0 is north
                
        // translate to x and y position
        outline.translate((int)x, (int)y);
                
        // do the same for the altHeadingPt
        altHeadingPt.setLocation(200, 0);
        rotate(altHeadingPt, altHeading + Math.PI * 0.5);               // change heading so 0 is north
        altHeadingPt.translate((int)x, (int)y);
                
        }
        
    public void fillCircle(Graphics2D g, int x, int y, int radius)
        {
        g.fillOval(x-radius, y-radius, 2*radius, 2*radius);
                
        }

    public void draw(Object object, Graphics2D g, DrawInfo2D info)
        {
        transformOutline();
                
        g.setColor(Color.LIGHT_GRAY);
        g.fillPolygon(outline);
        g.setColor(Color.BLACK);
        g.drawPolygon(outline);
                
        fillCircle(g, (int)x, (int)y, 7);
                
        if (showAltHeading)
            {
            g.setColor(Color.BLUE);
            g.drawLine((int)x, (int)y, altHeadingPt.x, altHeadingPt.y);
            }

        // draw path history
        Stroke temp = g.getStroke();
        g.setStroke(pathStroke);
        g.setColor(Color.BLUE);
        g.draw(pathHistory);
        g.setStroke(temp);      
        }

    public String getName(LocationWrapper wrapper)
        {
        return null;
        }

    public String getStatus(LocationWrapper wrapper)
        {
        return null;
        }

    public boolean setSelected(LocationWrapper wrapper, boolean selected)
        {
        return false;
        }

    public Inspector getInspector(LocationWrapper wrapper, GUIState state)
        {
        return null;
        }

    public Point2D.Double getPosition()
        {
        return new Point2D.Double(x, y);
        }

    }
