package com.purepursuit.control;

/**
 * A PID (Proportional-Integral-Derivative) controller implementation.
 * 
 * <p>This controller calculates a control output based on the error between
 * a setpoint and the measured process variable. It uses three terms:
 * <ul>
 *   <li><b>Proportional (kP):</b> Responds to current error</li>
 *   <li><b>Integral (kI):</b> Responds to accumulated error over time</li>
 *   <li><b>Derivative (kD):</b> Responds to rate of change of error</li>
 * </ul>
 * 
 * <p>The controller includes features like:
 * <ul>
 *   <li>Integral windup prevention with configurable limits</li>
 *   <li>Output clamping to prevent actuator saturation</li>
 *   <li>Tolerance checking for determining when target is reached</li>
 * </ul>
 * 
 * <p>Example usage:
 * <pre>
 * PIDController pid = new PIDController(0.1, 0.01, 0.05);
 * pid.setSetpoint(100.0);
 * pid.setOutputLimits(-1.0, 1.0);
 * 
 * // In your control loop:
 * double output = pid.calculate(currentPosition);
 * motor.setPower(output);
 * </pre>
 * 
 * @author PurePursuitFTC
 * @version 1.0
 */
public class PIDController {
    private double kP;
    private double kI;
    private double kD;
    
    private double setpoint;
    private double lastError;
    private double integral;
    private long lastTime;
    
    private double minOutput = -1.0;
    private double maxOutput = 1.0;
    
    private double integralLimit = Double.POSITIVE_INFINITY;
    private double tolerance = 0.0;
    private double derivativeTolerance = Double.POSITIVE_INFINITY;
    
    /**
     * Constructs a new PIDController with the specified gains.
     * 
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     */
    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.setpoint = 0.0;
        this.lastError = 0.0;
        this.integral = 0.0;
        this.lastTime = System.nanoTime();
    }
    
    /**
     * Calculates the control output for the given measurement.
     * 
     * @param measurement the current measured value
     * @return the calculated control output
     */
    public double calculate(double measurement) {
        return calculate(measurement, setpoint);
    }
    
    /**
     * Calculates the control output for the given measurement and setpoint.
     * This method allows for on-the-fly setpoint changes without resetting the controller.
     * 
     * @param measurement the current measured value
     * @param setpoint the desired setpoint
     * @return the calculated control output
     */
    public double calculate(double measurement, double setpoint) {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9; // Convert to seconds
        lastTime = currentTime;
        
        // Prevent division by zero or negative time steps
        if (dt <= 0) {
            dt = 0.02; // Default to 20ms
        }
        
        double error = setpoint - measurement;
        
        // Proportional term
        double proportional = kP * error;
        
        // Integral term with anti-windup
        integral += error * dt;
        
        // Clamp integral to prevent windup
        if (integral > integralLimit) {
            integral = integralLimit;
        } else if (integral < -integralLimit) {
            integral = -integralLimit;
        }
        
        double integralTerm = kI * integral;
        
        // Derivative term
        double derivative = (error - lastError) / dt;
        double derivativeTerm = kD * derivative;
        
        lastError = error;
        
        // Calculate output
        double output = proportional + integralTerm + derivativeTerm;
        
        // Clamp output to limits
        if (output > maxOutput) {
            output = maxOutput;
        } else if (output < minOutput) {
            output = minOutput;
        }
        
        return output;
    }
    
    /**
     * Sets the PID gains.
     * 
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     */
    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    
    /**
     * Sets the setpoint (target value).
     * 
     * @param setpoint the desired setpoint
     */
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }
    
    /**
     * Gets the current setpoint.
     * 
     * @return the current setpoint
     */
    public double getSetpoint() {
        return setpoint;
    }
    
    /**
     * Sets the output limits to prevent actuator saturation.
     * 
     * @param minOutput the minimum output value
     * @param maxOutput the maximum output value
     */
    public void setOutputLimits(double minOutput, double maxOutput) {
        if (minOutput > maxOutput) {
            throw new IllegalArgumentException("minOutput must be less than maxOutput");
        }
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }
    
    /**
     * Sets the integral windup limit.
     * This prevents the integral term from accumulating beyond a certain value.
     * 
     * @param limit the maximum absolute value of the integral
     */
    public void setIntegralLimit(double limit) {
        this.integralLimit = Math.abs(limit);
    }
    
    /**
     * Sets the tolerance for determining if the controller has reached the setpoint.
     * 
     * @param tolerance the acceptable error tolerance
     */
    public void setTolerance(double tolerance) {
        this.tolerance = Math.abs(tolerance);
    }
    
    /**
     * Sets the derivative tolerance for determining if the rate of change is acceptable.
     * 
     * @param derivativeTolerance the acceptable rate of change tolerance
     */
    public void setDerivativeTolerance(double derivativeTolerance) {
        this.derivativeTolerance = Math.abs(derivativeTolerance);
    }
    
    /**
     * Checks if the controller has reached the setpoint within the specified tolerance.
     * 
     * @return true if at setpoint, false otherwise
     */
    public boolean atSetpoint() {
        return Math.abs(lastError) <= tolerance;
    }
    
    /**
     * Checks if the controller has reached the setpoint within both position and derivative tolerances.
     * 
     * @param currentMeasurement the current measured value for derivative calculation
     * @return true if at setpoint with acceptable derivative, false otherwise
     */
    public boolean atSetpoint(double currentMeasurement) {
        double error = setpoint - currentMeasurement;
        double errorRate = Math.abs(error - lastError);
        return Math.abs(error) <= tolerance && errorRate <= derivativeTolerance;
    }
    
    /**
     * Resets the controller state.
     * Clears the integral accumulator and error history.
     */
    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        lastTime = System.nanoTime();
    }
    
    /**
     * Gets the current error (setpoint - measurement).
     * 
     * @return the last calculated error
     */
    public double getError() {
        return lastError;
    }
    
    /**
     * Gets the accumulated integral term.
     * 
     * @return the current integral value
     */
    public double getIntegral() {
        return integral;
    }
    
    /**
     * Gets the proportional gain.
     * 
     * @return kP value
     */
    public double getkP() {
        return kP;
    }
    
    /**
     * Gets the integral gain.
     * 
     * @return kI value
     */
    public double getkI() {
        return kI;
    }
    
    /**
     * Gets the derivative gain.
     * 
     * @return kD value
     */
    public double getkD() {
        return kD;
    }
}
