package com.purepursuit.pathing;

import com.purepursuit.geometry.Point2D;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a path as a series of waypoints.
 * 
 * <p>A path consists of multiple waypoints that the robot should follow.
 * Each waypoint has a position, and the path can be followed using
 * path-following algorithms like Pure Pursuit.
 * 
 * <p>Example usage:
 * <pre>
 * Path path = new Path();
 * path.addWaypoint(new Waypoint(0, 0, 10.0));
 * path.addWaypoint(new Waypoint(24, 24, 8.0));
 * path.addWaypoint(new Waypoint(48, 0, 6.0));
 * </pre>
 * 
 * @author PurePursuitFTC
 * @version 1.0
 */
public class Path {
    private final List<Waypoint> waypoints;
    private int lastFoundIndex;
    
    /**
     * Constructs an empty path.
     */
    public Path() {
        this.waypoints = new ArrayList<>();
        this.lastFoundIndex = 0;
    }
    
    /**
     * Constructs a path with the given waypoints.
     * 
     * @param waypoints the initial waypoints
     */
    public Path(List<Waypoint> waypoints) {
        this.waypoints = new ArrayList<>(waypoints);
        this.lastFoundIndex = 0;
    }
    
    /**
     * Adds a waypoint to the end of the path.
     * 
     * @param waypoint the waypoint to add
     * @return this path for method chaining
     */
    public Path addWaypoint(Waypoint waypoint) {
        waypoints.add(waypoint);
        return this;
    }
    
    /**
     * Adds a waypoint at the specified position with a default lookahead distance.
     * 
     * @param x the x-coordinate
     * @param y the y-coordinate
     * @return this path for method chaining
     */
    public Path addWaypoint(double x, double y) {
        return addWaypoint(new Waypoint(x, y));
    }
    
    /**
     * Adds a waypoint at the specified position with a custom lookahead distance.
     * 
     * @param x the x-coordinate
     * @param y the y-coordinate
     * @param lookaheadDistance the lookahead distance for this waypoint
     * @return this path for method chaining
     */
    public Path addWaypoint(double x, double y, double lookaheadDistance) {
        return addWaypoint(new Waypoint(x, y, lookaheadDistance));
    }
    
    /**
     * Gets the waypoint at the specified index.
     * 
     * @param index the index
     * @return the waypoint at the index
     */
    public Waypoint getWaypoint(int index) {
        return waypoints.get(index);
    }
    
    /**
     * Gets the number of waypoints in the path.
     * 
     * @return the number of waypoints
     */
    public int size() {
        return waypoints.size();
    }
    
    /**
     * Checks if the path is empty.
     * 
     * @return true if the path has no waypoints
     */
    public boolean isEmpty() {
        return waypoints.isEmpty();
    }
    
    /**
     * Gets all waypoints in the path.
     * 
     * @return a list of all waypoints
     */
    public List<Waypoint> getWaypoints() {
        return new ArrayList<>(waypoints);
    }
    
    /**
     * Gets the last waypoint in the path.
     * 
     * @return the last waypoint, or null if the path is empty
     */
    public Waypoint getLastWaypoint() {
        if (waypoints.isEmpty()) {
            return null;
        }
        return waypoints.get(waypoints.size() - 1);
    }
    
    /**
     * Gets the first waypoint in the path.
     * 
     * @return the first waypoint, or null if the path is empty
     */
    public Waypoint getFirstWaypoint() {
        if (waypoints.isEmpty()) {
            return null;
        }
        return waypoints.get(0);
    }
    
    /**
     * Clears all waypoints from the path.
     */
    public void clear() {
        waypoints.clear();
        lastFoundIndex = 0;
    }
    
    /**
     * Gets the last found index during path following.
     * This is used internally by the Pure Pursuit algorithm.
     * 
     * @return the last found index
     */
    public int getLastFoundIndex() {
        return lastFoundIndex;
    }
    
    /**
     * Sets the last found index during path following.
     * This is used internally by the Pure Pursuit algorithm.
     * 
     * @param index the index to set
     */
    public void setLastFoundIndex(int index) {
        this.lastFoundIndex = index;
    }
    
    @Override
    public String toString() {
        return "Path{waypoints=" + waypoints.size() + "}";
    }
}
