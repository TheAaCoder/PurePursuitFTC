package com.purepursuit.geometry;

import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Unit tests for Point2D class.
 */
public class Point2DTest {
    
    private static final double DELTA = 1e-6;
    
    @Test
    public void testConstruction() {
        Point2D point = new Point2D(3.0, 4.0);
        assertEquals(3.0, point.x, DELTA);
        assertEquals(4.0, point.y, DELTA);
    }
    
    @Test
    public void testDistanceTo() {
        Point2D p1 = new Point2D(0, 0);
        Point2D p2 = new Point2D(3, 4);
        assertEquals(5.0, p1.distanceTo(p2), DELTA);
    }
    
    @Test
    public void testAdd() {
        Point2D p1 = new Point2D(1, 2);
        Point2D p2 = new Point2D(3, 4);
        Point2D result = p1.add(p2);
        assertEquals(4.0, result.x, DELTA);
        assertEquals(6.0, result.y, DELTA);
    }
    
    @Test
    public void testSubtract() {
        Point2D p1 = new Point2D(5, 7);
        Point2D p2 = new Point2D(2, 3);
        Point2D result = p1.subtract(p2);
        assertEquals(3.0, result.x, DELTA);
        assertEquals(4.0, result.y, DELTA);
    }
    
    @Test
    public void testMultiply() {
        Point2D p = new Point2D(3, 4);
        Point2D result = p.multiply(2.0);
        assertEquals(6.0, result.x, DELTA);
        assertEquals(8.0, result.y, DELTA);
    }
    
    @Test
    public void testMagnitude() {
        Point2D p = new Point2D(3, 4);
        assertEquals(5.0, p.magnitude(), DELTA);
    }
    
    @Test
    public void testNormalize() {
        Point2D p = new Point2D(3, 4);
        Point2D normalized = p.normalize();
        assertEquals(1.0, normalized.magnitude(), DELTA);
        assertEquals(0.6, normalized.x, DELTA);
        assertEquals(0.8, normalized.y, DELTA);
    }
    
    @Test
    public void testNormalizeZeroVector() {
        Point2D p = new Point2D(0, 0);
        Point2D normalized = p.normalize();
        assertEquals(0.0, normalized.x, DELTA);
        assertEquals(0.0, normalized.y, DELTA);
    }
    
    @Test
    public void testDot() {
        Point2D p1 = new Point2D(2, 3);
        Point2D p2 = new Point2D(4, 5);
        assertEquals(23.0, p1.dot(p2), DELTA);
    }
    
    @Test
    public void testEquals() {
        Point2D p1 = new Point2D(1.0, 2.0);
        Point2D p2 = new Point2D(1.0, 2.0);
        Point2D p3 = new Point2D(1.0, 3.0);
        
        assertEquals(p1, p2);
        assertNotEquals(p1, p3);
    }
    
    @Test
    public void testToString() {
        Point2D p = new Point2D(1.5, 2.5);
        String str = p.toString();
        assertTrue(str.contains("1.50"));
        assertTrue(str.contains("2.50"));
    }
}
