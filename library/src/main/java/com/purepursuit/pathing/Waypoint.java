package com.purepursuit.pathing;

import com.purepursuit.geometry.Point2D;

/**
 * Represents a waypoint in a path.
 * 
 * <p>A waypoint includes a position and optional parameters like lookahead distance
 * that control how the Pure Pursuit algorithm behaves when approaching this point.
 * 
 * <p>Example usage:
 * <pre>
 * Waypoint wp1 = new Waypoint(10, 20);
 * Waypoint wp2 = new Waypoint(30, 40, 15.0); // Custom lookahead
 * </pre>
 * 
 * @author PurePursuitFTC
 * @version 1.0
 */
public class Waypoint {
    private final Point2D position;
    private final double lookaheadDistance;
    
    /**
     * Default lookahead distance in inches.
     */
    public static final double DEFAULT_LOOKAHEAD = 12.0;
    
    /**
     * Constructs a waypoint at the specified position with default lookahead distance.
     * 
     * @param x the x-coordinate
     * @param y the y-coordinate
     */
    public Waypoint(double x, double y) {
        this(x, y, DEFAULT_LOOKAHEAD);
    }
    
    /**
     * Constructs a waypoint at the specified position with custom lookahead distance.
     * 
     * @param x the x-coordinate
     * @param y the y-coordinate
     * @param lookaheadDistance the lookahead distance for this waypoint
     */
    public Waypoint(double x, double y, double lookaheadDistance) {
        this.position = new Point2D(x, y);
        this.lookaheadDistance = lookaheadDistance;
    }
    
    /**
     * Constructs a waypoint from a Point2D with default lookahead distance.
     * 
     * @param position the position
     */
    public Waypoint(Point2D position) {
        this(position, DEFAULT_LOOKAHEAD);
    }
    
    /**
     * Constructs a waypoint from a Point2D with custom lookahead distance.
     * 
     * @param position the position
     * @param lookaheadDistance the lookahead distance for this waypoint
     */
    public Waypoint(Point2D position, double lookaheadDistance) {
        this.position = position;
        this.lookaheadDistance = lookaheadDistance;
    }
    
    /**
     * Gets the position of this waypoint.
     * 
     * @return the position as a Point2D
     */
    public Point2D getPosition() {
        return position;
    }
    
    /**
     * Gets the x-coordinate of this waypoint.
     * 
     * @return the x-coordinate
     */
    public double getX() {
        return position.x;
    }
    
    /**
     * Gets the y-coordinate of this waypoint.
     * 
     * @return the y-coordinate
     */
    public double getY() {
        return position.y;
    }
    
    /**
     * Gets the lookahead distance for this waypoint.
     * 
     * @return the lookahead distance
     */
    public double getLookaheadDistance() {
        return lookaheadDistance;
    }
    
    /**
     * Calculates the distance from this waypoint to another point.
     * 
     * @param point the other point
     * @return the distance
     */
    public double distanceTo(Point2D point) {
        return position.distanceTo(point);
    }
    
    @Override
    public String toString() {
        return String.format("Waypoint(x=%.2f, y=%.2f, lookahead=%.2f)", 
            position.x, position.y, lookaheadDistance);
    }
}
