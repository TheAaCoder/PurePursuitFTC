# PurePursuitFTC Usage Guide

## Table of Contents
1. [Installation](#installation)
2. [Quick Start](#quick-start)
3. [Core Concepts](#core-concepts)
4. [Detailed Examples](#detailed-examples)
5. [Tuning Guide](#tuning-guide)
6. [Troubleshooting](#troubleshooting)

## Installation

### Method 1: Copy Source Files (Recommended for FTC)

1. Copy the entire `com.purepursuit` package to your TeamCode module:
   ```
   TeamCode/src/main/java/com/purepursuit/
   ```

2. Add FTCLib to your `build.gradle` (TeamCode module):
   ```gradle
   dependencies {
       implementation 'com.arcrobotics:ftclib:2.1.1'
       // ... other dependencies
   }
   ```

3. Sync your project

### Method 2: As a Gradle Module

Include the library module in your settings.gradle and add it as a dependency.

## Quick Start

### 1. Create Your Drive Subsystem

```java
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.purepursuit.subsystems.MecanumDriveSubsystem;

public class MyRobot extends CommandOpMode {
    private MecanumDriveSubsystem drive;
    
    @Override
    public void initialize() {
        // Initialize motors
        Motor fl = new Motor(hardwareMap, "frontLeft");
        Motor fr = new Motor(hardwareMap, "frontRight");
        Motor bl = new Motor(hardwareMap, "backLeft");
        Motor br = new Motor(hardwareMap, "backRight");
        
        // Invert right side motors (adjust for your robot)
        fr.setInverted(true);
        br.setInverted(true);
        
        // Create subsystem
        drive = new MecanumDriveSubsystem(fl, fr, bl, br);
        
        // Register subsystem
        register(drive);
    }
}
```

### 2. Create a Simple Path

```java
import com.purepursuit.pathing.Path;

Path simplePath = new Path()
    .addWaypoint(0, 0, 12.0)      // Start (x, y, lookahead)
    .addWaypoint(24, 24, 12.0)    // First waypoint
    .addWaypoint(48, 0, 12.0);    // End waypoint
```

### 3. Follow the Path

```java
import com.purepursuit.commands.FollowPathCommand;

FollowPathCommand followPath = new FollowPathCommand(drive, simplePath)
    .setForwardSpeed(0.5)
    .setMaxTurnSpeed(0.4);

schedule(followPath);
```

## Core Concepts

### Coordinate System

- **X-axis**: Positive is to the right
- **Y-axis**: Positive is forward
- **Heading**: Positive is counter-clockwise (standard math convention)
- **Units**: Inches for distance, radians for angles

### Pure Pursuit Algorithm

Pure Pursuit works by:
1. Finding where a "lookahead circle" intersects with your path
2. Calculating the curvature needed to reach that intersection point
3. Commanding the robot to follow that curvature

**Key Parameter: Lookahead Distance**
- Small (6-10"): Tight following, more oscillation
- Medium (10-15"): Balanced, good for most uses
- Large (15-20"): Smooth, cuts corners

### PID Control

The library uses PID controllers for:
- Heading correction (in path following)
- Position control (in drive-to-point)

PID formula: `output = kP*error + kI*∫error + kD*d(error)/dt`

## Detailed Examples

### Example 1: Complex Autonomous Routine

```java
@Autonomous(name = "Complex Auto")
public class ComplexAuto extends CommandOpMode {
    private MecanumDriveSubsystem drive;
    
    @Override
    public void initialize() {
        // Setup drive (see Quick Start section)
        drive = setupDrive();
        
        // Create multiple paths
        Path pathToFirstTarget = new Path()
            .addWaypoint(0, 0, 15.0)
            .addWaypoint(12, 12, 12.0)
            .addWaypoint(24, 12, 10.0);
        
        Path pathToSecondTarget = new Path()
            .addWaypoint(24, 12, 15.0)
            .addWaypoint(36, 24, 12.0)
            .addWaypoint(48, 24, 10.0);
        
        // Create commands
        FollowPathCommand path1 = new FollowPathCommand(drive, pathToFirstTarget)
            .setForwardSpeed(0.6)
            .setMaxTurnSpeed(0.4);
        
        DriveToPointCommand precisePosition = new DriveToPointCommand(
            drive, 
            new Point2D(24, 12), 
            0  // Face forward
        ).setPositionTolerance(1.0);
        
        FollowPathCommand path2 = new FollowPathCommand(drive, pathToSecondTarget)
            .setForwardSpeed(0.5);
        
        // Chain commands
        SequentialCommandGroup auto = new SequentialCommandGroup(
            path1,                      // Follow first path
            precisePosition,            // Position precisely
            new WaitCommand(1000),      // Wait 1 second
            path2                       // Follow second path
        );
        
        schedule(auto);
        register(drive);
    }
    
    @Override
    public void run() {
        // CRITICAL: Update pose with your odometry
        // Pose2D pose = yourOdometry.getPose();
        // drive.updatePose(pose);
        
        super.run();
    }
}
```

### Example 2: Field-Centric TeleOp

```java
@TeleOp(name = "Field Centric")
public class FieldCentricTeleOp extends CommandOpMode {
    
    @Override
    public void initialize() {
        MecanumDriveSubsystem drive = setupDrive();
        
        // Set default command for continuous driving
        drive.setDefaultCommand(new RunCommand(() -> {
            // Get gamepad input
            double strafe = -gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            
            // Apply deadzone
            strafe = applyDeadzone(strafe, 0.1);
            forward = applyDeadzone(forward, 0.1);
            turn = applyDeadzone(turn, 0.1);
            
            // Get heading (from IMU or odometry)
            double heading = drive.getHeading();
            
            // Drive field-centric
            drive.driveFieldCentric(strafe, forward, turn, heading);
        }, drive));
        
        register(drive);
    }
    
    private double applyDeadzone(double value, double deadzone) {
        return Math.abs(value) < deadzone ? 0 : value;
    }
}
```

### Example 3: Custom Command

```java
public class DriveToScoringPosition extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final PIDController xController;
    private final PIDController yController;
    private final Point2D targetPosition;
    
    public DriveToScoringPosition(MecanumDriveSubsystem drive) {
        this.drive = drive;
        this.targetPosition = new Point2D(36, 48); // Your scoring position
        
        this.xController = new PIDController(0.05, 0.0, 0.01);
        this.yController = new PIDController(0.05, 0.0, 0.01);
        
        xController.setSetpoint(targetPosition.x);
        yController.setSetpoint(targetPosition.y);
        
        addRequirements(drive);
    }
    
    @Override
    public void execute() {
        Pose2D pose = drive.getPose();
        
        double xOutput = xController.calculate(pose.x);
        double yOutput = yController.calculate(pose.y);
        
        drive.driveRobotCentric(xOutput, yOutput, 0);
    }
    
    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
```

## Tuning Guide

### Step 1: Tune Lookahead Distance

Start with 12 inches and adjust:

```java
Path path = new Path()
    .addWaypoint(0, 0, 12.0);  // Try different values: 8, 12, 16

// Or tune the controller directly
controller.setLookaheadDistance(12.0);
```

**Signs you need to adjust:**
- Robot oscillates: **Decrease** lookahead
- Robot cuts corners: **Increase** lookahead
- Robot goes off path: **Adjust** path waypoints or decrease speed

### Step 2: Tune Speed

```java
FollowPathCommand cmd = new FollowPathCommand(drive, path)
    .setForwardSpeed(0.5)      // Start at 0.5, adjust 0.3-0.8
    .setMaxTurnSpeed(0.4);     // Keep 0.3-0.5 to prevent tipping
```

### Step 3: Tune Heading PID

Default values: kP=1.0, kI=0.0, kD=0.1

```java
cmd.setHeadingPID(1.2, 0.0, 0.15);  // Increase P for faster response
```

**Tuning process:**
1. Start with P only: (1.0, 0.0, 0.0)
2. Increase P until oscillation
3. Add D to dampen: (P, 0.0, 0.1)
4. Usually no I needed

### Step 4: Tune DriveToPoint PID

```java
DriveToPointCommand cmd = new DriveToPointCommand(drive, target, heading)
    .setPositionPID(0.05, 0.001, 0.01)  // X and Y position
    .setHeadingPID(1.0, 0.0, 0.1);      // Heading
```

**Position PID tuning:**
1. Start: (0.05, 0, 0)
2. Increase kP if too slow
3. Add kD if oscillating: (0.05, 0, 0.01)
4. Add small kI if steady-state error: (0.05, 0.001, 0.01)

### Tuning Tips

- **Always tune on actual robot**, not in simulator
- Test at match speeds
- Tune one parameter at a time
- Document your final values
- Different paths may need different settings

## Troubleshooting

### Robot doesn't move

**Check:**
1. Are motors inverted correctly?
   ```java
   fr.setInverted(true);
   br.setInverted(true);
   ```

2. Is pose being updated?
   ```java
   @Override
   public void run() {
       drive.updatePose(odometry.getPose());  // REQUIRED!
       super.run();
   }
   ```

3. Is command scheduled?
   ```java
   schedule(followPathCommand);
   ```

### Robot oscillates

**Solutions:**
- Increase lookahead distance
- Lower forward speed
- Tune heading PID (reduce kP, increase kD)
- Check that pose updates are accurate

### Robot cuts corners

**Solutions:**
- Decrease lookahead distance
- Add more intermediate waypoints
- Slow down on turns (use different lookahead per waypoint)

### Robot drifts off path

**Check:**
1. Odometry accuracy - this is the #1 cause
2. Coordinate system orientation
3. Motor directions
4. Path waypoint coordinates

### Command never ends

**Check:**
- `isAtPathEnd()` threshold: increase if too strict
- Tolerances in DriveToPointCommand
- That robot is actually reaching target

### Build errors

**FTCLib/SDK not found:**
- Ensure FTCLib is in your build.gradle
- Check internet connection for dependency download
- Sync Gradle project

**Compilation errors:**
- Ensure Java 8 compatibility
- Check that all files are in correct packages
- Verify FTC SDK version compatibility

## Integration with Odometry

The library requires continuous pose updates. Here are integration examples:

### With Dead Wheel Odometry

```java
private Odometry odometry;

@Override
public void initialize() {
    odometry = new Odometry(/* your encoders */);
    // ... setup drive
}

@Override
public void run() {
    odometry.update();
    drive.updatePose(odometry.getPose());
    super.run();
}
```

### With Roadrunner

```java
private SampleMecanumDrive roadrunnerDrive;

@Override
public void run() {
    roadrunnerDrive.update();
    Pose2d rrPose = roadrunnerDrive.getPoseEstimate();
    
    // Convert to our Pose2D
    Pose2D pose = new Pose2D(
        rrPose.getX(), 
        rrPose.getY(), 
        rrPose.getHeading()
    );
    
    drive.updatePose(pose);
    super.run();
}
```

### With IMU + Drive Encoders

```java
@Override
public void run() {
    // Calculate position from encoders
    double x = calculateXFromEncoders();
    double y = calculateYFromEncoders();
    
    // Get heading from IMU
    double heading = imu.getAngularOrientation().firstAngle;
    
    drive.updatePose(new Pose2D(x, y, heading));
    super.run();
}
```

## Best Practices

1. **Always test incrementally**
   - Test motors first
   - Test odometry accuracy
   - Test simple paths before complex ones

2. **Use telemetry for debugging**
   ```java
   telemetry.addData("Robot X", drive.getPose().x);
   telemetry.addData("Robot Y", drive.getPose().y);
   telemetry.addData("Heading", Math.toDegrees(drive.getPose().heading));
   telemetry.update();
   ```

3. **Create reusable paths**
   ```java
   public class AutonomousPaths {
       public static Path getRedLeftPath() {
           return new Path()
               .addWaypoint(0, 0, 15.0)
               .addWaypoint(24, 24, 12.0);
       }
   }
   ```

4. **Handle interruptions**
   ```java
   @Override
   public void end(boolean interrupted) {
       if (interrupted) {
           // Handle early termination
       }
       drive.stop();
   }
   ```

5. **Log your tuning values**
   - Keep a document with your final PID gains
   - Document lookahead distances that work
   - Note speed settings for different scenarios

## Advanced Topics

### Velocity Control

```java
// Direct velocity control
drive.driveVelocity(vx, vy, omega);
```

### Multiple Subsystems

```java
ParallelCommandGroup simultaneous = new ParallelCommandGroup(
    new FollowPathCommand(drive, path),
    new IntakeCommand(intake),
    new PrepareShooterCommand(shooter)
);
```

### State Machines

```java
SequentialCommandGroup stateMachine = new SequentialCommandGroup(
    new DriveToPickupCommand(drive),
    new IntakeCommand(intake).withTimeout(2000),
    new DriveToScoringCommand(drive),
    new ScoreCommand(scorer).withTimeout(1000)
);
```

## Further Reading

- [FTCLib Documentation](https://docs.ftclib.org/)
- [Pure Pursuit Paper](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)
- [PID Control Tutorial](https://en.wikipedia.org/wiki/PID_controller)

## Support

For issues or questions:
1. Check this guide first
2. Review the Javadoc comments in the source code
3. Test with the provided examples
4. Check FTC Discord channels for community support
