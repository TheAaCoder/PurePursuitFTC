package com.purepursuit.examples;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.purepursuit.subsystems.MecanumDriveSubsystem;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Example TeleOp OpMode demonstrating field-centric drive with the Pure Pursuit library.
 * 
 * <p>This example shows:
 * <ul>
 *   <li>Setting up field-centric drive control</li>
 *   <li>Using FTCLib's GamepadEx for input</li>
 *   <li>Integrating with the command system for teleop</li>
 * </ul>
 * 
 * <p>Controls:
 * <ul>
 *   <li>Left stick: Strafe and forward/backward movement</li>
 *   <li>Right stick X: Rotation</li>
 * </ul>
 * 
 * @author PurePursuitFTC
 * @version 1.0
 */
@TeleOp(name = "Field Centric Drive Example", group = "Examples")
public class ExampleTeleOp extends CommandOpMode {
    
    private MecanumDriveSubsystem drive;
    private GamepadEx driverGamepad;
    
    @Override
    public void initialize() {
        // Initialize motors
        Motor frontLeft = new Motor(hardwareMap, "frontLeft");
        Motor frontRight = new Motor(hardwareMap, "frontRight");
        Motor backLeft = new Motor(hardwareMap, "backLeft");
        Motor backRight = new Motor(hardwareMap, "backRight");
        
        // Invert motors if necessary
        frontRight.setInverted(true);
        backRight.setInverted(true);
        
        // Create drive subsystem
        drive = new MecanumDriveSubsystem(frontLeft, frontRight, backLeft, backRight);
        
        // Initialize gamepad
        driverGamepad = new GamepadEx(gamepad1);
        
        // Set the default command for the drive subsystem
        // This command runs whenever no other command is using the drive subsystem
        drive.setDefaultCommand(new RunCommand(() -> {
            // Get joystick inputs
            double strafe = -driverGamepad.getLeftX();
            double forward = -driverGamepad.getLeftY();
            double turn = -driverGamepad.getRightX();
            
            // Apply deadzone
            if (Math.abs(strafe) < 0.1) strafe = 0;
            if (Math.abs(forward) < 0.1) forward = 0;
            if (Math.abs(turn) < 0.1) turn = 0;
            
            // Get current heading for field-centric drive
            // Note: Replace with actual IMU heading
            double heading = drive.getHeading();
            
            // Drive field-centric
            drive.driveFieldCentric(strafe, forward, turn, heading);
            
        }, drive));
        
        // Register the drive subsystem
        register(drive);
    }
    
    @Override
    public void run() {
        // IMPORTANT: Update robot pose with your odometry system
        // Example placeholder - replace with your actual odometry:
        // Pose2D pose = odometry.getPose();
        // drive.updatePose(pose);
        
        // Run the command scheduler
        super.run();
        
        // Telemetry
        telemetry.addData("Robot X", drive.getPose().x);
        telemetry.addData("Robot Y", drive.getPose().y);
        telemetry.addData("Robot Heading (deg)", Math.toDegrees(drive.getPose().heading));
        telemetry.addData("Left Stick X", driverGamepad.getLeftX());
        telemetry.addData("Left Stick Y", driverGamepad.getLeftY());
        telemetry.addData("Right Stick X", driverGamepad.getRightX());
        telemetry.update();
    }
}
