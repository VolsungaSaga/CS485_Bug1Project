/*
 * MapDisplay.java
 *
 * By Joey Harrison
 * Last Modified 12/8/07
 * 
 * This is the base-level graphical component for drawing a map. It draws a list
 * of items that implement the Portrayal2D interface.
 * 
 * It also handles mouse drags and scrolls to perform pans and zooms, respectively.
 * 
 * The following improvements will be useful in the future:
 *              - buffered drawing of layers so map layers can be redrawn only when necessary
 *              - maintain and service a mode which will take mouse and keyboard events
 *              - add function to zoom to extents
 *              - draw a zoom interface on the screen (think google maps)
 *
 */

package gmu.robot.pioneer.jgui;

import javax.swing.JComponent;
import javax.vecmath.Vector2d;

import sim.portrayal.DrawInfo2D;
import sim.portrayal.Portrayal2D;

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

public class MapDisplay extends JComponent implements MouseListener, MouseMotionListener, MouseWheelListener, 
                                           HierarchyBoundsListener
    {
    private static final long serialVersionUID = 1L;

    Point mousePos = new Point();
    Point prevMousePos = new Point();
    Point screenCenter = new Point();
        
    Rectangle2D.Double boundary = new Rectangle2D.Double();
    private double pixelsPerMeter = 1.0;
    private double metersPerPixel = 1.0;
        
    /*
     * These are the objects to be drawn.
     */
    ArrayList<Portrayal2D> portrayals = new ArrayList<Portrayal2D>();
        
    /*
     * Before drawing, the screen will center on this object if it's not null.
     */
    private Poseable2D objectToFollow = null;
        
    public MapDisplay()
        {
        this.addMouseListener(this);
        this.addMouseMotionListener(this);
        this.addMouseWheelListener(this);
        this.addHierarchyBoundsListener(this);
        }

    public void paint(Graphics g)
        {
        Graphics2D g2 = (Graphics2D)g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
                
        //TODO lock on this object here
        if (objectToFollow != null)
            {
            centerBoundaryOn(boundary, objectToFollow.getPosition());
            }

        int width = getWidth();
        int height = getHeight();

        g2.translate(width/2, height/2);
        g2.scale(pixelsPerMeter, -pixelsPerMeter);
        g2.translate(-boundary.getCenterX(), -boundary.getCenterY());
                
        //TODO This is currently sending dummy parameters to the Portrayal2D. 
        // Either send them proper values, or drop the Portrayal2D pretense.
                
        // portrayals need a drawing/clipping rectangle. Give them this one for now.
        Rectangle2D.Double tempRect = new Rectangle2D.Double(0, 0, (double)width, (double)height);
                
        for (int i = 0; i < portrayals.size(); i++)
            portrayals.get(i).draw(null, g2, new DrawInfo2D(tempRect, tempRect));

        /*
          g2.setColor(Color.BLUE);
          g2.drawLine(0, 0, width, height);
          g2.drawLine(width, 0, 0, height);
        
          g2.setColor(Color.BLACK);
          g2.drawRect(0, 0, width, height);
        */
        }

    public void mouseClicked(MouseEvent e)
        {
        }

    public void mouseEntered(MouseEvent e)
        {
        }

    public void mouseExited(MouseEvent e)
        {
        }

    public void mousePressed(MouseEvent e)
        {
        }

    public void mouseReleased(MouseEvent e)
        {
        }

    public void mouseDragged(MouseEvent e)
        {
        setObjectToFollow(null);
                
        mousePos = e.getPoint();
                
        double deltaX = (mousePos.x - prevMousePos.x) * metersPerPixel; 
        double deltaY = (mousePos.y - prevMousePos.y) * metersPerPixel; 
        boundary.x -= deltaX;
        boundary.y += deltaY;

        prevMousePos = mousePos;
                
        repaint();
        }
        
    /*
     * This should be improved later to notify listeners.
     */
    private void reportMousePosition()
        {
        Point2D.Double worldPos = screenToWorld(mousePos.x, mousePos.y);

        System.out.format("Screen: (%d, %d)  World: (%.2f, %.2f)\n", 
            mousePos.x, mousePos.y, worldPos.x, worldPos.y);
        }

    public void mouseMoved(MouseEvent e)
        {       
        mousePos = e.getPoint();
        prevMousePos = mousePos;
                
        //reportMousePosition();
        }

    public void mouseWheelMoved(MouseWheelEvent e)
        {
        // down scrolls are positive, up scrolls are negative
        int delta = e.getUnitsToScroll();
        Point2D.Double pos = screenToWorld(mousePos.x, mousePos.y);
                
        zoom(pos.x, pos.y, delta);
        repaint();
        }
        
    public void ancestorMoved(HierarchyEvent arg0)
        {
        }

    public void ancestorResized(HierarchyEvent arg0)
        {
        boundary.width = this.getWidth() * metersPerPixel;
        boundary.height = this.getHeight() * metersPerPixel;
        }
        
    public void setObjectToFollow(Poseable2D obj)
        {
        //TODO lock on this object here
        objectToFollow = obj;
        }
        
    private void centerBoundaryOn(Rectangle2D.Double b, Point2D.Double p)
        {
        b.x = p.x - b.width * 0.5;
        b.y = p.y - b.height * 0.5;
        }
        
    private void scaleBoundary(double factor)
        {
        double centerX = boundary.getCenterX();
        double centerY = boundary.getCenterY();

        double cornerX = centerX + boundary.width * 0.5 * factor;
        double cornerY = centerY + boundary.height * 0.5 * factor;
                
        boundary.setFrameFromCenter(centerX, centerY, cornerX, cornerY);
        }

    /*
     * Convert the given screen coordinates to world coordinates.
     */
    public Point2D.Double screenToWorld(int screenX, int screenY)
        {
        double worldX = boundary.getMinX() + screenX * metersPerPixel;
        double worldY = boundary.getMinY() + (getHeight() - screenY) * metersPerPixel;
                
        return new Point2D.Double(worldX, worldY);
        }
        
    /*
     * Convert the given screen coordinates to world coordinates.
     */
    public Point2D.Double screenToWorld(Point p)
        {
        return screenToWorld(p.x, p.y);
        }
        
    /*
     * Convert the given world coordinates to screen coordinates.
     */
    public Point worldToScreen(Point2D.Double worldP)
        {       
        int x = (int)Math.round((worldP.x - boundary.getMinX()) * pixelsPerMeter);
        int y = getHeight() - (int)Math.round((worldP.y - boundary.getMinY()) * pixelsPerMeter);
                
        return new Point(x, y);
        }
        
    private void updateScreenDimensions()
        {       
        double metersPerPixelX = boundary.width / getWidth();           
        double metersPerPixelY = boundary.height / getHeight();
                
        metersPerPixel = Math.max(metersPerPixelX, metersPerPixelY);
        pixelsPerMeter = 1.0 / metersPerPixel;
        }
        
    /*
     * Change the zoom based on a mouse scroll. The boundary will be scaled
     * and moved to keep the mouse over the same point.
     */
    private void zoom(double x, double y, int delta)
        {
        double zoomPercent = 0.01;
        double zoomFactor = 1.0;
                
        if (delta < 0) // zoom in
            zoomFactor = 1.0 - zoomPercent * -delta;
        else    // zoom out
            zoomFactor = 1.0 / (1.0 - zoomPercent * delta);

        scaleBoundary(zoomFactor);

        // calculate a vector to translate the boundary such that the
        // mouse stays in the same place.
        Vector2d v = new Vector2d(x - boundary.getCenterX(), y - boundary.getCenterY());
        v.scale(1.0 - zoomFactor);
        boundary.x += v.x;
        boundary.y += v.y;
                
        updateScreenDimensions();
        }
        
    public void zoom(double factor)
        {
        scaleBoundary(factor);
        updateScreenDimensions();
        }
    }
