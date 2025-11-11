package com.purepursuit.pathing;

import com.purepursuit.geometry.Point2D;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Unit tests for Path and Waypoint classes.
 */
public class PathTest {
    
    private static final double DELTA = 1e-6;
    
    @Test
    public void testWaypointConstruction() {
        Waypoint wp = new Waypoint(10, 20);
        assertEquals(10.0, wp.getX(), DELTA);
        assertEquals(20.0, wp.getY(), DELTA);
        assertEquals(Waypoint.DEFAULT_LOOKAHEAD, wp.getLookaheadDistance(), DELTA);
    }
    
    @Test
    public void testWaypointWithCustomLookahead() {
        Waypoint wp = new Waypoint(10, 20, 15.0);
        assertEquals(15.0, wp.getLookaheadDistance(), DELTA);
    }
    
    @Test
    public void testPathConstruction() {
        Path path = new Path();
        assertTrue(path.isEmpty());
        assertEquals(0, path.size());
    }
    
    @Test
    public void testAddWaypoint() {
        Path path = new Path();
        path.addWaypoint(0, 0);
        path.addWaypoint(10, 10);
        
        assertEquals(2, path.size());
        assertFalse(path.isEmpty());
    }
    
    @Test
    public void testGetWaypoint() {
        Path path = new Path();
        path.addWaypoint(5, 10, 12.0);
        path.addWaypoint(15, 20, 15.0);
        
        Waypoint wp = path.getWaypoint(0);
        assertEquals(5.0, wp.getX(), DELTA);
        assertEquals(10.0, wp.getY(), DELTA);
    }
    
    @Test
    public void testGetFirstLastWaypoint() {
        Path path = new Path();
        path.addWaypoint(0, 0);
        path.addWaypoint(10, 10);
        path.addWaypoint(20, 20);
        
        Waypoint first = path.getFirstWaypoint();
        Waypoint last = path.getLastWaypoint();
        
        assertEquals(0.0, first.getX(), DELTA);
        assertEquals(20.0, last.getX(), DELTA);
    }
    
    @Test
    public void testClear() {
        Path path = new Path();
        path.addWaypoint(0, 0);
        path.addWaypoint(10, 10);
        
        path.clear();
        assertTrue(path.isEmpty());
        assertEquals(0, path.size());
    }
    
    @Test
    public void testWaypointDistanceTo() {
        Waypoint wp = new Waypoint(0, 0);
        Point2D point = new Point2D(3, 4);
        assertEquals(5.0, wp.distanceTo(point), DELTA);
    }
}
