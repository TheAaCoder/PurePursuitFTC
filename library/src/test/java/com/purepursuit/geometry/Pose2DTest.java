package com.purepursuit.geometry;

import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Unit tests for Pose2D class.
 */
public class Pose2DTest {
    
    private static final double DELTA = 1e-6;
    
    @Test
    public void testConstruction() {
        Pose2D pose = new Pose2D(3.0, 4.0, Math.PI / 4);
        assertEquals(3.0, pose.x, DELTA);
        assertEquals(4.0, pose.y, DELTA);
        assertEquals(Math.PI / 4, pose.heading, DELTA);
    }
    
    @Test
    public void testConstructionFromPoint() {
        Point2D point = new Point2D(5, 6);
        Pose2D pose = new Pose2D(point, Math.PI / 2);
        assertEquals(5.0, pose.x, DELTA);
        assertEquals(6.0, pose.y, DELTA);
        assertEquals(Math.PI / 2, pose.heading, DELTA);
    }
    
    @Test
    public void testGetPosition() {
        Pose2D pose = new Pose2D(3, 4, 0);
        Point2D pos = pose.getPosition();
        assertEquals(3.0, pos.x, DELTA);
        assertEquals(4.0, pos.y, DELTA);
    }
    
    @Test
    public void testDistanceTo() {
        Pose2D p1 = new Pose2D(0, 0, 0);
        Pose2D p2 = new Pose2D(3, 4, Math.PI);
        assertEquals(5.0, p1.distanceTo(p2), DELTA);
    }
    
    @Test
    public void testNormalizeAngle() {
        assertEquals(0.0, Pose2D.normalizeAngle(0), DELTA);
        assertEquals(Math.PI, Pose2D.normalizeAngle(Math.PI), DELTA);
        assertEquals(-Math.PI, Pose2D.normalizeAngle(-Math.PI), DELTA);
        
        // Test wrapping
        double angle = 3 * Math.PI; // Should wrap to PI
        double normalized = Pose2D.normalizeAngle(angle);
        assertTrue(normalized > -Math.PI && normalized <= Math.PI);
    }
    
    @Test
    public void testHeadingDifference() {
        double diff = Pose2D.headingDifference(Math.PI / 2, 0);
        assertEquals(Math.PI / 2, diff, DELTA);
        
        // Test wrap-around
        double diff2 = Pose2D.headingDifference(-Math.PI * 0.9, Math.PI * 0.9);
        assertTrue(Math.abs(diff2) <= Math.PI);
    }
    
    @Test
    public void testSubtract() {
        Pose2D p1 = new Pose2D(5, 7, Math.PI / 2);
        Pose2D p2 = new Pose2D(2, 3, Math.PI / 4);
        Pose2D result = p1.subtract(p2);
        assertEquals(3.0, result.x, DELTA);
        assertEquals(4.0, result.y, DELTA);
        assertEquals(Math.PI / 4, result.heading, DELTA);
    }
    
    @Test
    public void testEquals() {
        Pose2D p1 = new Pose2D(1.0, 2.0, 0.5);
        Pose2D p2 = new Pose2D(1.0, 2.0, 0.5);
        Pose2D p3 = new Pose2D(1.0, 2.0, 0.6);
        
        assertEquals(p1, p2);
        assertNotEquals(p1, p3);
    }
}
