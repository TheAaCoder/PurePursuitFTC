package com.purepursuit.geometry;

/**
 * Represents a 2D point in the coordinate system.
 * This class is immutable and thread-safe.
 * 
 * <p>Example usage:
 * <pre>
 * Point2D point = new Point2D(10.5, 20.3);
 * double distance = point.distanceTo(new Point2D(5.0, 15.0));
 * </pre>
 * 
 * @author PurePursuitFTC
 * @version 1.0
 */
public class Point2D {
    /** The x-coordinate of the point */
    public final double x;
    
    /** The y-coordinate of the point */
    public final double y;
    
    /**
     * Constructs a new Point2D with the specified coordinates.
     * 
     * @param x the x-coordinate
     * @param y the y-coordinate
     */
    public Point2D(double x, double y) {
        this.x = x;
        this.y = y;
    }
    
    /**
     * Calculates the Euclidean distance between this point and another point.
     * 
     * @param other the other point to calculate distance to
     * @return the distance between the two points
     */
    public double distanceTo(Point2D other) {
        double dx = this.x - other.x;
        double dy = this.y - other.y;
        return Math.sqrt(dx * dx + dy * dy);
    }
    
    /**
     * Adds another point to this point and returns a new point.
     * 
     * @param other the point to add
     * @return a new Point2D representing the sum
     */
    public Point2D add(Point2D other) {
        return new Point2D(this.x + other.x, this.y + other.y);
    }
    
    /**
     * Subtracts another point from this point and returns a new point.
     * 
     * @param other the point to subtract
     * @return a new Point2D representing the difference
     */
    public Point2D subtract(Point2D other) {
        return new Point2D(this.x - other.x, this.y - other.y);
    }
    
    /**
     * Multiplies this point by a scalar value.
     * 
     * @param scalar the scalar value to multiply by
     * @return a new Point2D representing the scaled point
     */
    public Point2D multiply(double scalar) {
        return new Point2D(this.x * scalar, this.y * scalar);
    }
    
    /**
     * Calculates the magnitude (length) of the vector from origin to this point.
     * 
     * @return the magnitude of the vector
     */
    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }
    
    /**
     * Returns a normalized (unit length) version of this point as a vector.
     * If the magnitude is zero, returns a zero vector.
     * 
     * @return a normalized Point2D
     */
    public Point2D normalize() {
        double mag = magnitude();
        if (mag == 0) {
            return new Point2D(0, 0);
        }
        return new Point2D(x / mag, y / mag);
    }
    
    /**
     * Calculates the dot product with another point (treated as vectors).
     * 
     * @param other the other point
     * @return the dot product
     */
    public double dot(Point2D other) {
        return this.x * other.x + this.y * other.y;
    }
    
    @Override
    public String toString() {
        return String.format("Point2D(x=%.2f, y=%.2f)", x, y);
    }
    
    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        Point2D other = (Point2D) obj;
        return Double.compare(x, other.x) == 0 && Double.compare(y, other.y) == 0;
    }
    
    @Override
    public int hashCode() {
        long xBits = Double.doubleToLongBits(x);
        long yBits = Double.doubleToLongBits(y);
        return (int) (xBits ^ (xBits >>> 32) ^ yBits ^ (yBits >>> 32));
    }
}
