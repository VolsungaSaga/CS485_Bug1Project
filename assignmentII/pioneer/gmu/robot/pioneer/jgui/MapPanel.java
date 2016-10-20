/*
 * MapPanel.java
 *
 * By Joey Harrison
 * Last Modified 12/8/07
 * 
 * This panel holds a MapDisplay and adds a panel beneath it which prints the
 * current mouse position.
 *
 */

package gmu.robot.pioneer.jgui;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Point;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionListener;
import java.awt.geom.Point2D;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JLabel;
import javax.swing.JPanel;

public class MapPanel extends JPanel implements MouseMotionListener
    {
    private static final long serialVersionUID = 1L;
    MapDisplay mapDisplay;
    JPanel textPanel;
    JLabel lblMousePos;

    public MapPanel()
        {
        this.setBorder(BorderFactory.createLineBorder(Color.black));
        this.setLayout(new BorderLayout());

        mapDisplay = new MapDisplay();
        add(mapDisplay, BorderLayout.CENTER);
        
        mapDisplay.addMouseMotionListener(this);
        
        textPanel = new JPanel();
        textPanel.setPreferredSize(new Dimension(400, 20));
        textPanel.setLayout(new BoxLayout(textPanel, BoxLayout.X_AXIS));
        textPanel.setBorder(BorderFactory.createLineBorder(Color.gray));
        
        add(textPanel, BorderLayout.SOUTH);
        
        lblMousePos = new JLabel("X: XXXXX.XX  Y: XXXXX.XX");

        textPanel.add(Box.createHorizontalGlue());
        textPanel.add(lblMousePos);
        textPanel.add(Box.createRigidArea(new Dimension(20, 0)));
        }
        
    private void updateMousePosition(Point p)
        {
        Point2D.Double worldP = mapDisplay.screenToWorld(p.x, p.y);
        lblMousePos.setText(String.format("X: %-7.2f  Y: %-7.2f", worldP.x, worldP.y)); 
        }

    public void mouseDragged(MouseEvent e)
        {
        updateMousePosition(e.getPoint());
        }

    public void mouseMoved(MouseEvent e)
        {
        updateMousePosition(e.getPoint());
        }
    }
