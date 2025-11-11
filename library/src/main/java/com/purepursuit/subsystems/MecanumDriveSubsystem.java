package com.purepursuit.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.purepursuit.geometry.Pose2D;
import com.purepursuit.geometry.Point2D;

/**
 * A subsystem for a mecanum drive robot that integrates with FTCLib's command system.
 * 
 * <p>This subsystem provides high-level control for a mecanum drive robot, including:
 * <ul>
 *   <li>Field-centric and robot-centric driving</li>
 *   <li>Odometry tracking (requires external pose updates)</li>
 *   <li>Integration with Pure Pursuit path following</li>
 * </ul>
 * 
 * <p>Example usage:
 * <pre>
 * Motor fl = new Motor(hardwareMap, "frontLeft");
 * Motor fr = new Motor(hardwareMap, "frontRight");
 * Motor bl = new Motor(hardwareMap, "backLeft");
 * Motor br = new Motor(hardwareMap, "backRight");
 * 
 * MecanumDriveSubsystem drive = new MecanumDriveSubsystem(fl, fr, bl, br);
 * 
 * // Use in teleop
 * drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, robotHeading);
 * 
 * // Use with commands
 * CommandScheduler.getInstance().schedule(
 *     new FollowPathCommand(drive, path)
 * );
 * </pre>
 * 
 * @author PurePursuitFTC
 * @version 1.0
 */
public class MecanumDriveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;
    private Pose2D currentPose;
    
    /**
     * Constructs a MecanumDriveSubsystem with the specified motors.
     * 
     * @param frontLeft the front left motor
     * @param frontRight the front right motor
     * @param backLeft the back left motor
     * @param backRight the back right motor
     */
    public MecanumDriveSubsystem(Motor frontLeft, Motor frontRight, 
                                 Motor backLeft, Motor backRight) {
        this.drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        this.currentPose = new Pose2D(0, 0, 0);
    }
    
    /**
     * Drives the robot using robot-centric control.
     * 
     * @param strafeSpeed the strafe speed (-1.0 to 1.0, positive is right)
     * @param forwardSpeed the forward speed (-1.0 to 1.0, positive is forward)
     * @param turnSpeed the turn speed (-1.0 to 1.0, positive is counter-clockwise)
     */
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }
    
    /**
     * Drives the robot using field-centric control.
     * 
     * @param strafeSpeed the strafe speed (-1.0 to 1.0, positive is right)
     * @param forwardSpeed the forward speed (-1.0 to 1.0, positive is forward)
     * @param turnSpeed the turn speed (-1.0 to 1.0, positive is counter-clockwise)
     * @param gyroAngle the current gyro angle in radians
     */
    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, 
                                 double turnSpeed, double gyroAngle) {
        drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, gyroAngle);
    }
    
    /**
     * Drives the robot with specified velocities for each direction.
     * Useful for Pure Pursuit and other autonomous driving.
     * 
     * @param vx the velocity in the x direction (strafe)
     * @param vy the velocity in the y direction (forward)
     * @param omega the angular velocity (turn rate)
     */
    public void driveVelocity(double vx, double vy, double omega) {
        driveRobotCentric(vx, vy, omega);
    }
    
    /**
     * Stops the robot by setting all motor powers to zero.
     */
    public void stop() {
        drive.stop();
    }
    
    /**
     * Updates the robot's current pose.
     * This should be called regularly with odometry updates.
     * 
     * @param pose the new pose
     */
    public void updatePose(Pose2D pose) {
        this.currentPose = pose;
    }
    
    /**
     * Gets the robot's current pose.
     * 
     * @return the current pose
     */
    public Pose2D getPose() {
        return currentPose;
    }
    
    /**
     * Resets the robot's pose to the origin.
     */
    public void resetPose() {
        this.currentPose = new Pose2D(0, 0, 0);
    }
    
    /**
     * Sets the robot's pose to a specific value.
     * Useful for setting the initial pose at the start of autonomous.
     * 
     * @param x the x-coordinate
     * @param y the y-coordinate
     * @param heading the heading in radians
     */
    public void setPose(double x, double y, double heading) {
        this.currentPose = new Pose2D(x, y, heading);
    }
    
    /**
     * Gets the robot's current position.
     * 
     * @return the current position
     */
    public Point2D getPosition() {
        return currentPose.getPosition();
    }
    
    /**
     * Gets the robot's current heading.
     * 
     * @return the current heading in radians
     */
    public double getHeading() {
        return currentPose.heading;
    }
    
    @Override
    public void periodic() {
        // This method is called periodically by the CommandScheduler
        // Can be used for telemetry or continuous updates
    }
}
