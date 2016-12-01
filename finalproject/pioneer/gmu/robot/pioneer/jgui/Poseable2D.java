/*
 * Poseable2D.java
 *
 * By Joey Harrison
 * Last Modified 12/8/07
 * 
 * This simple interface was created so MapDisplay can center on a robot without
 * being coupled to any specific robot class. This can easily be discarded if  a
 * suitable replacement already exists in the standard java library.
 *
 */

package gmu.robot.pioneer.jgui;

import java.awt.geom.Point2D;

public interface Poseable2D
    {
    public Point2D.Double getPosition();
    }
