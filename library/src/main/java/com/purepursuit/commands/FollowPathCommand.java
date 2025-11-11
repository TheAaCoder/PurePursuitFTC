package com.purepursuit.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.purepursuit.control.PIDController;
import com.purepursuit.geometry.Point2D;
import com.purepursuit.geometry.Pose2D;
import com.purepursuit.pathing.Path;
import com.purepursuit.pathing.PurePursuitController;
import com.purepursuit.subsystems.MecanumDriveSubsystem;

/**
 * A command that follows a path using the Pure Pursuit algorithm.
 * 
 * <p>This command integrates with FTCLib's command system and uses:
 * <ul>
 *   <li>Pure Pursuit for lateral control (steering)</li>
 *   <li>PID for heading control</li>
 *   <li>Configurable forward speed</li>
 * </ul>
 * 
 * <p>The command ends when the robot reaches the end of the path.
 * 
 * <p>Example usage:
 * <pre>
 * Path path = new Path()
 *     .addWaypoint(0, 0, 15.0)
 *     .addWaypoint(24, 24, 12.0)
 *     .addWaypoint(48, 0, 10.0);
 * 
 * FollowPathCommand command = new FollowPathCommand(driveSubsystem, path);
 * command.setForwardSpeed(0.6);
 * 
 * CommandScheduler.getInstance().schedule(command);
 * </pre>
 * 
 * @author PurePursuitFTC
 * @version 1.0
 */
public class FollowPathCommand extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final Path path;
    private final PurePursuitController purePursuit;
    private final PIDController headingController;
    
    private double forwardSpeed = 0.5;
    private double maxTurnSpeed = 0.4;
    
    /**
     * Constructs a FollowPathCommand with default Pure Pursuit parameters.
     * 
     * @param drive the drive subsystem
     * @param path the path to follow
     */
    public FollowPathCommand(MecanumDriveSubsystem drive, Path path) {
        this(drive, path, 12.0);
    }
    
    /**
     * Constructs a FollowPathCommand with custom lookahead distance.
     * 
     * @param drive the drive subsystem
     * @param path the path to follow
     * @param lookaheadDistance the lookahead distance in inches
     */
    public FollowPathCommand(MecanumDriveSubsystem drive, Path path, double lookaheadDistance) {
        this.drive = drive;
        this.path = path;
        this.purePursuit = new PurePursuitController(lookaheadDistance);
        this.headingController = new PIDController(1.0, 0.0, 0.1);
        
        addRequirements(drive);
    }
    
    @Override
    public void initialize() {
        // Reset path tracking
        path.setLastFoundIndex(0);
        headingController.reset();
    }
    
    @Override
    public void execute() {
        Pose2D robotPose = drive.getPose();
        
        // Get the lookahead point from Pure Pursuit
        Point2D lookaheadPoint = purePursuit.getLookaheadPoint(path, robotPose);
        
        // Calculate curvature to reach the lookahead point
        double curvature = purePursuit.calculateCurvature(robotPose, lookaheadPoint);
        
        // Calculate the target heading from the lookahead point
        Point2D robotPos = robotPose.getPosition();
        double dx = lookaheadPoint.x - robotPos.x;
        double dy = lookaheadPoint.y - robotPos.y;
        double targetHeading = Math.atan2(dy, dx);
        
        // Use PID to control heading
        double headingError = Pose2D.headingDifference(targetHeading, robotPose.heading);
        double turnSpeed = headingController.calculate(-headingError, 0);
        
        // Clamp turn speed
        if (turnSpeed > maxTurnSpeed) {
            turnSpeed = maxTurnSpeed;
        } else if (turnSpeed < -maxTurnSpeed) {
            turnSpeed = -maxTurnSpeed;
        }
        
        // Calculate strafe component based on curvature
        double strafeSpeed = curvature * forwardSpeed * 2.0;
        
        // Drive the robot
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }
    
    @Override
    public boolean isFinished() {
        return purePursuit.isAtPathEnd(path, drive.getPose());
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
    
    /**
     * Sets the forward speed for path following.
     * 
     * @param speed the forward speed (0.0 to 1.0)
     * @return this command for method chaining
     */
    public FollowPathCommand setForwardSpeed(double speed) {
        this.forwardSpeed = Math.abs(speed);
        return this;
    }
    
    /**
     * Sets the maximum turn speed.
     * 
     * @param speed the maximum turn speed (0.0 to 1.0)
     * @return this command for method chaining
     */
    public FollowPathCommand setMaxTurnSpeed(double speed) {
        this.maxTurnSpeed = Math.abs(speed);
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
    public FollowPathCommand setHeadingPID(double kP, double kI, double kD) {
        headingController.setPID(kP, kI, kD);
        return this;
    }
    
    /**
     * Sets the lookahead distance for Pure Pursuit.
     * 
     * @param distance the lookahead distance in inches
     * @return this command for method chaining
     */
    public FollowPathCommand setLookaheadDistance(double distance) {
        purePursuit.setLookaheadDistance(distance);
        return this;
    }
}
