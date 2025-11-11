package com.purepursuit.examples;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.purepursuit.commands.DriveToPointCommand;
import com.purepursuit.commands.FollowPathCommand;
import com.purepursuit.geometry.Point2D;
import com.purepursuit.pathing.Path;
import com.purepursuit.subsystems.MecanumDriveSubsystem;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Example autonomous OpMode demonstrating the Pure Pursuit library with FTCLib commands.
 * 
 * <p>This example shows:
 * <ul>
 *   <li>Setting up the drive subsystem with motors</li>
 *   <li>Creating paths with waypoints</li>
 *   <li>Using FollowPathCommand to follow paths</li>
 *   <li>Using DriveToPointCommand for precise positioning</li>
 *   <li>Chaining commands with SequentialCommandGroup</li>
 * </ul>
 * 
 * <p><b>Note:</b> This example assumes you have:
 * <ul>
 *   <li>An odometry system that updates the robot's pose</li>
 *   <li>Motors configured in your robot configuration as shown below</li>
 * </ul>
 * 
 * @author PurePursuitFTC
 * @version 1.0
 */
@Autonomous(name = "Pure Pursuit Example", group = "Examples")
public class ExampleAutonomous extends CommandOpMode {
    
    private MecanumDriveSubsystem drive;
    
    @Override
    public void initialize() {
        // Initialize motors
        // Note: Adjust motor names to match your robot configuration
        Motor frontLeft = new Motor(hardwareMap, "frontLeft");
        Motor frontRight = new Motor(hardwareMap, "frontRight");
        Motor backLeft = new Motor(hardwareMap, "backLeft");
        Motor backRight = new Motor(hardwareMap, "backRight");
        
        // Invert motors if necessary
        frontRight.setInverted(true);
        backRight.setInverted(true);
        
        // Create drive subsystem
        drive = new MecanumDriveSubsystem(frontLeft, frontRight, backLeft, backRight);
        
        // Set initial pose (starting position)
        // Adjust these values based on your robot's starting position
        drive.setPose(0, 0, 0);
        
        // Create a path with multiple waypoints
        Path path1 = new Path()
            .addWaypoint(0, 0, 15.0)      // Start position with 15" lookahead
            .addWaypoint(24, 24, 12.0)    // First turn with 12" lookahead
            .addWaypoint(48, 24, 10.0)    // Straight section with 10" lookahead
            .addWaypoint(48, 0, 12.0);    // Final position with 12" lookahead
        
        // Create a second path
        Path path2 = new Path()
            .addWaypoint(48, 0, 15.0)
            .addWaypoint(24, -24, 12.0)
            .addWaypoint(0, 0, 10.0);
        
        // Create commands for path following
        FollowPathCommand followPath1 = new FollowPathCommand(drive, path1)
            .setForwardSpeed(0.6)           // 60% forward speed
            .setMaxTurnSpeed(0.4)           // 40% max turn speed
            .setHeadingPID(1.2, 0.0, 0.15); // Tune PID for your robot
        
        FollowPathCommand followPath2 = new FollowPathCommand(drive, path2)
            .setForwardSpeed(0.5)
            .setMaxTurnSpeed(0.4);
        
        // Create a command to drive to a specific point with precise positioning
        DriveToPointCommand driveToEnd = new DriveToPointCommand(
            drive, 
            new Point2D(0, 0),  // Target position
            0                    // Target heading (0 radians)
        )
        .setPositionTolerance(1.0)           // Within 1" of target
        .setHeadingTolerance(Math.toRadians(3)) // Within 3 degrees
        .setMaxSpeed(0.4);                   // Slower for precision
        
        // Create a sequential command group that runs commands in order
        SequentialCommandGroup autoSequence = new SequentialCommandGroup(
            followPath1,                // Follow first path
            new WaitCommand(500),       // Wait 0.5 seconds
            followPath2,                // Follow second path
            new WaitCommand(500),       // Wait 0.5 seconds
            driveToEnd                  // Drive to final position precisely
        );
        
        // Schedule the autonomous sequence
        schedule(autoSequence);
        
        // Register the drive subsystem
        register(drive);
    }
    
    @Override
    public void run() {
        // IMPORTANT: You must update the robot's pose here using your odometry system
        // This example uses placeholder values - replace with your actual odometry
        
        // Example: If using dead-wheel odometry or a localization system:
        // Pose2D currentPose = odometry.getPose();
        // drive.updatePose(currentPose);
        
        // For testing without odometry, you can use motor encoders (less accurate):
        // double x = calculateXFromEncoders();
        // double y = calculateYFromEncoders();
        // double heading = imu.getAngularOrientation().firstAngle;
        // drive.updatePose(new Pose2D(x, y, heading));
        
        // Run the command scheduler
        CommandScheduler.getInstance().run();
        
        // Telemetry for debugging
        telemetry.addData("Robot X", drive.getPose().x);
        telemetry.addData("Robot Y", drive.getPose().y);
        telemetry.addData("Robot Heading (deg)", Math.toDegrees(drive.getPose().heading));
        telemetry.update();
    }
}
