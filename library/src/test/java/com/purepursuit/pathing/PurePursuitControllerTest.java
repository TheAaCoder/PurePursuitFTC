package com.purepursuit.pathing;

import com.purepursuit.geometry.Point2D;
import com.purepursuit.geometry.Pose2D;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Unit tests for PurePursuitController class.
 */
public class PurePursuitControllerTest {
    
    private static final double DELTA = 1e-6;
    private PurePursuitController controller;
    private Path path;
    
    @Before
    public void setUp() {
        controller = new PurePursuitController(10.0);
        path = new Path();
        path.addWaypoint(0, 0, 10.0);
        path.addWaypoint(20, 0, 10.0);
        path.addWaypoint(20, 20, 10.0);
    }
    
    @Test
    public void testConstruction() {
        assertEquals(10.0, controller.getLookaheadDistance(), DELTA);
        assertEquals(PurePursuitController.DEFAULT_END_THRESHOLD, 
                    controller.getEndPathThreshold(), DELTA);
    }
    
    @Test
    public void testSetLookaheadDistance() {
        controller.setLookaheadDistance(15.0);
        assertEquals(15.0, controller.getLookaheadDistance(), DELTA);
    }
    
    @Test
    public void testGetLookaheadPoint() {
        Pose2D robotPose = new Pose2D(0, 0, 0);
        Point2D lookahead = controller.getLookaheadPoint(path, robotPose);
        
        assertNotNull(lookahead);
        // Lookahead point should be ahead on the path
        assertTrue(lookahead.x >= 0);
    }
    
    @Test
    public void testCalculateCurvature() {
        Pose2D robotPose = new Pose2D(0, 0, 0);
        Point2D target = new Point2D(10, 5);
        
        double curvature = controller.calculateCurvature(robotPose, target);
        
        // Curvature should be positive for left turn
        assertTrue(curvature > 0);
    }
    
    @Test
    public void testCalculateCurvatureStraight() {
        Pose2D robotPose = new Pose2D(0, 0, 0);
        Point2D target = new Point2D(10, 0);
        
        double curvature = controller.calculateCurvature(robotPose, target);
        
        // Curvature should be near zero for straight line
        assertEquals(0.0, curvature, 0.1);
    }
    
    @Test
    public void testIsAtPathEnd() {
        Pose2D nearEnd = new Pose2D(20, 20, 0);
        Pose2D farFromEnd = new Pose2D(0, 0, 0);
        
        assertTrue(controller.isAtPathEnd(path, nearEnd));
        assertFalse(controller.isAtPathEnd(path, farFromEnd));
    }
    
    @Test
    public void testEmptyPath() {
        Path emptyPath = new Path();
        Pose2D pose = new Pose2D(0, 0, 0);
        
        assertTrue(controller.isAtPathEnd(emptyPath, pose));
        Point2D lookahead = controller.getLookaheadPoint(emptyPath, pose);
        assertEquals(pose.getPosition(), lookahead);
    }
}
