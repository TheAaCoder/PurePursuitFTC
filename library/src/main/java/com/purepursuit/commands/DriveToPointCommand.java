package com.purepursuit.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.purepursuit.control.PIDController;
import com.purepursuit.geometry.Point2D;
import com.purepursuit.geometry.Pose2D;
import com.purepursuit.subsystems.MecanumDriveSubsystem;

/**
 * A command that drives the robot to a target point using PID control.
 * 
 * <p>This command uses separate PID controllers for:
 * <ul>
 *   <li>X-axis position control</li>
 *   <li>Y-axis position control</li>
 *   <li>Heading control</li>
 * </ul>
 * 
 * <p>Example usage:
 * <pre>
 * DriveToPointCommand command = new DriveToPointCommand(
 *     driveSubsystem, 
 *     new Point2D(24, 36), 
 *     Math.PI / 2  // Face 90 degrees
 * );
 * 
 * command.setPositionTolerance(2.0);
 * command.setHeadingTolerance(Math.toRadians(5));
 * 
 * CommandScheduler.getInstance().schedule(command);
 * </pre>
 * 
 * @author PurePursuitFTC
 * @version 1.0
 */
public class DriveToPointCommand extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final Point2D targetPoint;
    private final double targetHeading;
    
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController headingController;
    
    private double positionTolerance = 2.0;
    private double headingTolerance = Math.toRadians(5);
    private double maxSpeed = 0.8;
    
    /**
     * Constructs a DriveToPointCommand that drives to a point while maintaining heading.
     * 
     * @param drive the drive subsystem
     * @param targetPoint the target point
     * @param targetHeading the target heading in radians
     */
    public DriveToPointCommand(MecanumDriveSubsystem drive, Point2D targetPoint, 
                              double targetHeading) {
        this.drive = drive;
        this.targetPoint = targetPoint;
        this.targetHeading = targetHeading;
        
        // Initialize PID controllers with default gains
        this.xController = new PIDController(0.05, 0.001, 0.01);
        this.yController = new PIDController(0.05, 0.001, 0.01);
        this.headingController = new PIDController(1.0, 0.0, 0.1);
        
        // Set output limits
        xController.setOutputLimits(-maxSpeed, maxSpeed);
        yController.setOutputLimits(-maxSpeed, maxSpeed);
        headingController.setOutputLimits(-0.5, 0.5);
        
        addRequirements(drive);
    }
    
    /**
     * Constructs a DriveToPointCommand that drives to a point with current heading.
     * 
     * @param drive the drive subsystem
     * @param targetPoint the target point
     */
    public DriveToPointCommand(MecanumDriveSubsystem drive, Point2D targetPoint) {
        this(drive, targetPoint, drive.getHeading());
    }
    
    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        headingController.reset();
        
        xController.setSetpoint(targetPoint.x);
        yController.setSetpoint(targetPoint.y);
        headingController.setSetpoint(targetHeading);
    }
    
    @Override
    public void execute() {
        Pose2D currentPose = drive.getPose();
        
        // Calculate control outputs
        double xSpeed = xController.calculate(currentPose.x);
        double ySpeed = yController.calculate(currentPose.y);
        double turnSpeed = headingController.calculate(currentPose.heading);
        
        // Convert to field-centric control
        double heading = currentPose.heading;
        double strafeSpeed = xSpeed * Math.cos(heading) + ySpeed * Math.sin(heading);
        double forwardSpeed = -xSpeed * Math.sin(heading) + ySpeed * Math.cos(heading);
        
        // Drive the robot
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }
    
    @Override
    public boolean isFinished() {
        Pose2D currentPose = drive.getPose();
        Point2D currentPos = currentPose.getPosition();
        
        double positionError = currentPos.distanceTo(targetPoint);
        double headingError = Math.abs(Pose2D.headingDifference(targetHeading, currentPose.heading));
        
        return positionError < positionTolerance && headingError < headingTolerance;
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
    
    /**
     * Sets the position tolerance for determining when the target is reached.
     * 
     * @param tolerance the position tolerance in inches
     * @return this command for method chaining
     */
    public DriveToPointCommand setPositionTolerance(double tolerance) {
        this.positionTolerance = tolerance;
        return this;
    }
    
    /**
     * Sets the heading tolerance for determining when the target is reached.
     * 
     * @param tolerance the heading tolerance in radians
     * @return this command for method chaining
     */
    public DriveToPointCommand setHeadingTolerance(double tolerance) {
        this.headingTolerance = tolerance;
        return this;
    }
    
    /**
     * Sets the maximum speed for movement.
     * 
     * @param speed the maximum speed (0.0 to 1.0)
     * @return this command for method chaining
     */
    public DriveToPointCommand setMaxSpeed(double speed) {
        this.maxSpeed = Math.abs(speed);
        xController.setOutputLimits(-maxSpeed, maxSpeed);
        yController.setOutputLimits(-maxSpeed, maxSpeed);
        return this;
    }
    
    /**
     * Sets the PID gains for position control (both X and Y).
     * 
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     * @return this command for method chaining
     */
    public DriveToPointCommand setPositionPID(double kP, double kI, double kD) {
        xController.setPID(kP, kI, kD);
        yController.setPID(kP, kI, kD);
        return this;
    }
    
    /**
     * Sets the PID gains for heading control.
     * 
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     * @return this command for method chaining
     */
    public DriveToPointCommand setHeadingPID(double kP, double kI, double kD) {
        headingController.setPID(kP, kI, kD);
        return this;
    }
}
