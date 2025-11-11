package com.purepursuit.geometry;

/**
 * Represents a 2D pose (position and heading) in the coordinate system.
 * This class is immutable and thread-safe.
 * 
 * <p>A pose consists of an (x, y) position and a heading angle in radians.
 * The heading is normalized to the range [-π, π].
 * 
 * <p>Example usage:
 * <pre>
 * Pose2D pose = new Pose2D(10.0, 20.0, Math.PI / 4);
 * Pose2D newPose = pose.add(new Pose2D(5.0, 5.0, 0));
 * </pre>
 * 
 * @author PurePursuitFTC
 * @version 1.0
 */
public class Pose2D {
    /** The x-coordinate of the pose */
    public final double x;
    
    /** The y-coordinate of the pose */
    public final double y;
    
    /** The heading angle in radians, normalized to [-π, π] */
    public final double heading;
    
    /**
     * Constructs a new Pose2D with the specified position and heading.
     * The heading is automatically normalized to [-π, π].
     * 
     * @param x the x-coordinate
     * @param y the y-coordinate
     * @param heading the heading angle in radians
     */
    public Pose2D(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = normalizeAngle(heading);
    }
    
    /**
     * Constructs a Pose2D from a Point2D and heading.
     * 
     * @param point the position
     * @param heading the heading angle in radians
     */
    public Pose2D(Point2D point, double heading) {
        this(point.x, point.y, heading);
    }
    
    /**
     * Returns the position component as a Point2D.
     * 
     * @return the position as a Point2D
     */
    public Point2D getPosition() {
        return new Point2D(x, y);
    }
    
    /**
     * Calculates the Euclidean distance between this pose and another pose.
     * Only considers the position, not the heading.
     * 
     * @param other the other pose
     * @return the distance between the two poses
     */
    public double distanceTo(Pose2D other) {
        return getPosition().distanceTo(other.getPosition());
    }
    
    /**
     * Adds another pose to this pose and returns a new pose.
     * This performs coordinate transformation.
     * 
     * @param other the pose to add
     * @return a new Pose2D representing the sum
     */
    public Pose2D add(Pose2D other) {
        double cos = Math.cos(this.heading);
        double sin = Math.sin(this.heading);
        
        double newX = this.x + other.x * cos - other.y * sin;
        double newY = this.y + other.x * sin + other.y * cos;
        double newHeading = this.heading + other.heading;
        
        return new Pose2D(newX, newY, newHeading);
    }
    
    /**
     * Subtracts another pose from this pose and returns a new pose.
     * 
     * @param other the pose to subtract
     * @return a new Pose2D representing the difference
     */
    public Pose2D subtract(Pose2D other) {
        return new Pose2D(
            this.x - other.x,
            this.y - other.y,
            this.heading - other.heading
        );
    }
    
    /**
     * Normalizes an angle to the range [-π, π].
     * 
     * @param angle the angle in radians
     * @return the normalized angle
     */
    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
    
    /**
     * Calculates the heading difference between two angles.
     * The result is in the range [-π, π].
     * 
     * @param targetHeading the target heading
     * @param currentHeading the current heading
     * @return the heading difference (target - current)
     */
    public static double headingDifference(double targetHeading, double currentHeading) {
        return normalizeAngle(targetHeading - currentHeading);
    }
    
    @Override
    public String toString() {
        return String.format("Pose2D(x=%.2f, y=%.2f, heading=%.2f rad)", x, y, heading);
    }
    
    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        Pose2D other = (Pose2D) obj;
        return Double.compare(x, other.x) == 0 
            && Double.compare(y, other.y) == 0 
            && Double.compare(heading, other.heading) == 0;
    }
    
    @Override
    public int hashCode() {
        long xBits = Double.doubleToLongBits(x);
        long yBits = Double.doubleToLongBits(y);
        long hBits = Double.doubleToLongBits(heading);
        return (int) (xBits ^ (xBits >>> 32) ^ yBits ^ (yBits >>> 32) ^ hBits ^ (hBits >>> 32));
    }
}
