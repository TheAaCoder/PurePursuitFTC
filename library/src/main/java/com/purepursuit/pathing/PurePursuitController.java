package com.purepursuit.pathing;

import com.purepursuit.geometry.Point2D;
import com.purepursuit.geometry.Pose2D;

/**
 * Implementation of the Pure Pursuit path following algorithm.
 * 
 * <p>Pure Pursuit is a path tracking algorithm that calculates the curvature
 * needed to reach a "lookahead point" on the path. The algorithm:
 * <ol>
 *   <li>Finds the closest point on the path to the robot</li>
 *   <li>Looks ahead along the path by a fixed distance (lookahead)</li>
 *   <li>Calculates the curvature needed to reach that lookahead point</li>
 *   <li>Commands the robot to drive along that curvature</li>
 * </ol>
 * 
 * <p>The lookahead distance is a tuning parameter that affects how the robot
 * follows the path:
 * <ul>
 *   <li>Smaller lookahead: Tighter following, but more oscillation</li>
 *   <li>Larger lookahead: Smoother following, but cuts corners more</li>
 * </ul>
 * 
 * <p>Example usage:
 * <pre>
 * PurePursuitController controller = new PurePursuitController(12.0);
 * Path path = new Path()
 *     .addWaypoint(0, 0)
 *     .addWaypoint(24, 24)
 *     .addWaypoint(48, 0);
 * 
 * // In your control loop:
 * Pose2D robotPose = getRobotPose();
 * Point2D target = controller.getLookaheadPoint(path, robotPose);
 * double curvature = controller.calculateCurvature(robotPose, target);
 * </pre>
 * 
 * @author PurePursuitFTC
 * @version 1.0
 */
public class PurePursuitController {
    private double lookaheadDistance;
    private double endPathThreshold;
    
    /**
     * Default threshold for determining when the robot has reached the end of the path.
     */
    public static final double DEFAULT_END_THRESHOLD = 3.0;
    
    /**
     * Constructs a Pure Pursuit controller with the specified lookahead distance.
     * 
     * @param lookaheadDistance the lookahead distance in inches
     */
    public PurePursuitController(double lookaheadDistance) {
        this(lookaheadDistance, DEFAULT_END_THRESHOLD);
    }
    
    /**
     * Constructs a Pure Pursuit controller with specified parameters.
     * 
     * @param lookaheadDistance the lookahead distance in inches
     * @param endPathThreshold the threshold distance for reaching the path end
     */
    public PurePursuitController(double lookaheadDistance, double endPathThreshold) {
        this.lookaheadDistance = lookaheadDistance;
        this.endPathThreshold = endPathThreshold;
    }
    
    /**
     * Finds the lookahead point on the path.
     * 
     * <p>This method searches for the intersection of a circle (centered at the robot
     * with radius = lookahead distance) with the path segments. It returns the
     * farthest intersection point found.
     * 
     * @param path the path to follow
     * @param robotPose the current robot pose
     * @return the lookahead point, or the last waypoint if at end of path
     */
    public Point2D getLookaheadPoint(Path path, Pose2D robotPose) {
        if (path.isEmpty()) {
            return robotPose.getPosition();
        }
        
        Point2D robotPos = robotPose.getPosition();
        Point2D lookaheadPoint = null;
        double maxDistance = 0;
        
        int startIndex = Math.max(0, path.getLastFoundIndex());
        
        // Search through path segments
        for (int i = startIndex; i < path.size() - 1; i++) {
            Waypoint start = path.getWaypoint(i);
            Waypoint end = path.getWaypoint(i + 1);
            
            // Use the lookahead distance from the waypoint if available
            double currentLookahead = start.getLookaheadDistance();
            if (currentLookahead <= 0) {
                currentLookahead = lookaheadDistance;
            }
            
            // Find intersection of circle with line segment
            Point2D intersection = lineCircleIntersection(
                start.getPosition(), 
                end.getPosition(), 
                robotPos, 
                currentLookahead
            );
            
            if (intersection != null) {
                double distanceAlongPath = robotPos.distanceTo(intersection);
                if (distanceAlongPath > maxDistance) {
                    maxDistance = distanceAlongPath;
                    lookaheadPoint = intersection;
                    path.setLastFoundIndex(i);
                }
            }
        }
        
        // If no intersection found, use the closest waypoint ahead
        if (lookaheadPoint == null) {
            lookaheadPoint = findClosestWaypointAhead(path, robotPos, startIndex);
        }
        
        return lookaheadPoint;
    }
    
    /**
     * Calculates the curvature needed to reach the target point.
     * 
     * <p>The curvature is calculated using the Pure Pursuit formula:
     * curvature = 2 * x / L^2
     * where x is the lateral distance to the target in the robot's frame,
     * and L is the distance to the target.
     * 
     * @param robotPose the current robot pose
     * @param targetPoint the target point to reach
     * @return the curvature (1/radius), positive for left turn, negative for right
     */
    public double calculateCurvature(Pose2D robotPose, Point2D targetPoint) {
        // Transform target point to robot's coordinate frame
        Point2D robotPos = robotPose.getPosition();
        double dx = targetPoint.x - robotPos.x;
        double dy = targetPoint.y - robotPos.y;
        
        // Rotate to robot frame
        double heading = robotPose.heading;
        double localX = dx * Math.cos(-heading) - dy * Math.sin(-heading);
        double localY = dx * Math.sin(-heading) + dy * Math.cos(-heading);
        
        // Calculate distance to target
        double distance = Math.sqrt(localX * localX + localY * localY);
        
        if (distance < 0.001) {
            return 0.0; // Prevent division by zero
        }
        
        // Pure Pursuit curvature formula: k = 2x / L^2
        double curvature = 2.0 * localY / (distance * distance);
        
        return curvature;
    }
    
    /**
     * Checks if the robot has reached the end of the path.
     * 
     * @param path the path being followed
     * @param robotPose the current robot pose
     * @return true if the robot is within the threshold of the last waypoint
     */
    public boolean isAtPathEnd(Path path, Pose2D robotPose) {
        if (path.isEmpty()) {
            return true;
        }
        
        Waypoint lastWaypoint = path.getLastWaypoint();
        double distance = robotPose.getPosition().distanceTo(lastWaypoint.getPosition());
        
        return distance < endPathThreshold;
    }
    
    /**
     * Finds the intersection of a line segment with a circle.
     * Returns the farthest intersection point along the segment.
     * 
     * @param segmentStart the start of the line segment
     * @param segmentEnd the end of the line segment
     * @param circleCenter the center of the circle
     * @param radius the radius of the circle
     * @return the intersection point, or null if no intersection
     */
    private Point2D lineCircleIntersection(Point2D segmentStart, Point2D segmentEnd, 
                                          Point2D circleCenter, double radius) {
        // Vector from start to end
        Point2D d = segmentEnd.subtract(segmentStart);
        
        // Vector from start to circle center
        Point2D f = segmentStart.subtract(circleCenter);
        
        double a = d.dot(d);
        double b = 2 * f.dot(d);
        double c = f.dot(f) - radius * radius;
        
        double discriminant = b * b - 4 * a * c;
        
        if (discriminant < 0) {
            return null; // No intersection
        }
        
        discriminant = Math.sqrt(discriminant);
        
        double t1 = (-b - discriminant) / (2 * a);
        double t2 = (-b + discriminant) / (2 * a);
        
        // Check if intersections are within the segment (0 <= t <= 1)
        Point2D intersection = null;
        
        if (t2 >= 0 && t2 <= 1) {
            // Use the farther intersection point
            intersection = segmentStart.add(d.multiply(t2));
        } else if (t1 >= 0 && t1 <= 1) {
            intersection = segmentStart.add(d.multiply(t1));
        }
        
        return intersection;
    }
    
    /**
     * Finds the closest waypoint ahead of the current position.
     * 
     * @param path the path
     * @param robotPos the robot position
     * @param startIndex the index to start searching from
     * @return the position of the closest waypoint ahead
     */
    private Point2D findClosestWaypointAhead(Path path, Point2D robotPos, int startIndex) {
        double minDistance = Double.POSITIVE_INFINITY;
        Point2D closest = path.getLastWaypoint().getPosition();
        
        for (int i = startIndex; i < path.size(); i++) {
            Point2D wp = path.getWaypoint(i).getPosition();
            double distance = robotPos.distanceTo(wp);
            
            if (distance < minDistance) {
                minDistance = distance;
                closest = wp;
            }
        }
        
        return closest;
    }
    
    /**
     * Sets the lookahead distance.
     * 
     * @param lookaheadDistance the new lookahead distance in inches
     */
    public void setLookaheadDistance(double lookaheadDistance) {
        this.lookaheadDistance = lookaheadDistance;
    }
    
    /**
     * Gets the current lookahead distance.
     * 
     * @return the lookahead distance in inches
     */
    public double getLookaheadDistance() {
        return lookaheadDistance;
    }
    
    /**
     * Sets the end path threshold.
     * 
     * @param threshold the threshold distance in inches
     */
    public void setEndPathThreshold(double threshold) {
        this.endPathThreshold = threshold;
    }
    
    /**
     * Gets the end path threshold.
     * 
     * @return the threshold distance in inches
     */
    public double getEndPathThreshold() {
        return endPathThreshold;
    }
}
