# PurePursuitFTC

A comprehensive FTC library that implements Pure Pursuit path following and PID control, fully integrated with FTCLib's command-based system.

## Features

- **Pure Pursuit Path Following**: Smooth path following algorithm with configurable lookahead distance
- **PID Controllers**: Well-tested PID implementation with anti-windup and output limiting
- **FTCLib Integration**: Full compatibility with FTCLib's command-based system
- **Comprehensive Documentation**: Detailed Javadocs and examples for all components
- **Mecanum Drive Support**: Pre-built subsystem for mecanum drive robots
- **Easy to Use**: Simple API with method chaining and sensible defaults

## Installation

### Adding to Your FTC Project

1. Clone this repository or download the library
2. Copy the `library/src/main/java/com/purepursuit` directory to your TeamCode module
3. Add FTCLib to your `build.gradle`:

```gradle
dependencies {
    implementation 'com.arcrobotics:ftclib:2.1.1'
    // ... other dependencies
}
```

## Quick Start

### Basic Path Following

```java
@Autonomous(name = "My Autonomous")
public class MyAutonomous extends CommandOpMode {
    private MecanumDriveSubsystem drive;
    
    @Override
    public void initialize() {
        // Initialize motors
        Motor fl = new Motor(hardwareMap, "frontLeft");
        Motor fr = new Motor(hardwareMap, "frontRight");
        Motor bl = new Motor(hardwareMap, "backLeft");
        Motor br = new Motor(hardwareMap, "backRight");
        
        // Create drive subsystem
        drive = new MecanumDriveSubsystem(fl, fr, bl, br);
        
        // Create a path
        Path path = new Path()
            .addWaypoint(0, 0, 15.0)
            .addWaypoint(24, 24, 12.0)
            .addWaypoint(48, 0, 10.0);
        
        // Create and schedule command
        FollowPathCommand followPath = new FollowPathCommand(drive, path)
            .setForwardSpeed(0.6)
            .setMaxTurnSpeed(0.4);
        
        schedule(followPath);
        register(drive);
    }
    
    @Override
    public void run() {
        // Update pose with your odometry system
        // drive.updatePose(odometry.getPose());
        
        CommandScheduler.getInstance().run();
    }
}
```

### Field-Centric TeleOp

```java
@TeleOp(name = "My TeleOp")
public class MyTeleOp extends CommandOpMode {
    @Override
    public void initialize() {
        Motor fl = new Motor(hardwareMap, "frontLeft");
        Motor fr = new Motor(hardwareMap, "frontRight");
        Motor bl = new Motor(hardwareMap, "backLeft");
        Motor br = new Motor(hardwareMap, "backRight");
        
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(fl, fr, bl, br);
        
        drive.setDefaultCommand(new RunCommand(() -> {
            double strafe = -gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double heading = drive.getHeading(); // Or use IMU
            
            drive.driveFieldCentric(strafe, forward, turn, heading);
        }, drive));
        
        register(drive);
    }
}
```

## Core Components

### Geometry Classes

#### Point2D
Represents a 2D point with utility methods for vector operations.

```java
Point2D p1 = new Point2D(10, 20);
Point2D p2 = new Point2D(5, 15);
double distance = p1.distanceTo(p2);
Point2D sum = p1.add(p2);
```

#### Pose2D
Represents a robot pose (position + heading).

```java
Pose2D pose = new Pose2D(10, 20, Math.PI / 4);
double x = pose.x;
double y = pose.y;
double heading = pose.heading; // in radians
```

### Control

#### PIDController
A robust PID controller with anti-windup and output limiting.

```java
PIDController pid = new PIDController(0.1, 0.01, 0.05);
pid.setSetpoint(100.0);
pid.setOutputLimits(-1.0, 1.0);
pid.setTolerance(2.0);

// In control loop
double output = pid.calculate(currentValue);
if (pid.atSetpoint()) {
    // Target reached
}
```

**Features:**
- Integral windup prevention
- Configurable output limits
- Position and derivative tolerance checking
- Automatic time step calculation

### Path Following

#### Path & Waypoint
Define paths as a series of waypoints.

```java
Path path = new Path()
    .addWaypoint(0, 0, 15.0)      // x, y, lookahead distance
    .addWaypoint(24, 24, 12.0)
    .addWaypoint(48, 0, 10.0);

// Or with Point2D
path.addWaypoint(new Waypoint(new Point2D(60, 0), 12.0));
```

#### PurePursuitController
Implements the Pure Pursuit algorithm.

```java
PurePursuitController controller = new PurePursuitController(12.0);
Point2D lookahead = controller.getLookaheadPoint(path, robotPose);
double curvature = controller.calculateCurvature(robotPose, lookahead);

if (controller.isAtPathEnd(path, robotPose)) {
    // Path complete
}
```

**Algorithm Details:**
- Finds intersection of lookahead circle with path segments
- Returns farthest intersection point
- Calculates curvature using formula: k = 2x / L²
- Handles path completion detection

### Subsystems

#### MecanumDriveSubsystem
FTCLib subsystem for mecanum drive robots.

```java
MecanumDriveSubsystem drive = new MecanumDriveSubsystem(fl, fr, bl, br);

// Robot-centric drive
drive.driveRobotCentric(strafe, forward, turn);

// Field-centric drive
drive.driveFieldCentric(strafe, forward, turn, heading);

// Update and query pose
drive.updatePose(new Pose2D(x, y, heading));
Pose2D current = drive.getPose();
```

### Commands

#### FollowPathCommand
Command for autonomous path following.

```java
FollowPathCommand cmd = new FollowPathCommand(drive, path)
    .setForwardSpeed(0.6)              // 0.0 to 1.0
    .setMaxTurnSpeed(0.4)              // Max turn rate
    .setLookaheadDistance(15.0)        // Override path lookahead
    .setHeadingPID(1.0, 0.0, 0.1);     // Tune heading control
```

#### DriveToPointCommand
Precise point-to-point movement with PID control.

```java
DriveToPointCommand cmd = new DriveToPointCommand(drive, target, heading)
    .setPositionTolerance(1.0)              // inches
    .setHeadingTolerance(Math.toRadians(3)) // radians
    .setMaxSpeed(0.5)
    .setPositionPID(0.05, 0.001, 0.01)
    .setHeadingPID(1.0, 0.0, 0.1);
```

### Chaining Commands

Use FTCLib's command groups to create complex autonomous routines:

```java
SequentialCommandGroup auto = new SequentialCommandGroup(
    new FollowPathCommand(drive, path1),
    new WaitCommand(500),
    new DriveToPointCommand(drive, new Point2D(24, 24), 0),
    new ParallelCommandGroup(
        new FollowPathCommand(drive, path2),
        new ArmToPositionCommand(arm, 1000)
    )
);

schedule(auto);
```

## Tuning Guide

### Pure Pursuit Lookahead Distance

The lookahead distance is the most important tuning parameter:

- **Smaller (6-10")**: Tighter path following, more oscillation, better for tight turns
- **Medium (10-15")**: Balanced performance, good for most paths
- **Larger (15-20")**: Smoother following, cuts corners more, better for high-speed runs

You can set different lookahead distances for different waypoints:

```java
Path path = new Path()
    .addWaypoint(0, 0, 15.0)    // Larger for straight sections
    .addWaypoint(24, 24, 8.0)   // Smaller for tight turns
    .addWaypoint(48, 24, 15.0);
```

### PID Tuning

#### Position Control (DriveToPointCommand)
1. Start with P-only: `kP = 0.05, kI = 0, kD = 0`
2. Increase P until oscillation occurs
3. Add D to dampen oscillation: `kD = 0.01`
4. Add I if steady-state error exists: `kI = 0.001`

#### Heading Control
1. Start with: `kP = 1.0, kI = 0, kD = 0.1`
2. Increase P for faster response
3. Increase D if overshooting
4. Usually no I term needed for heading

### Speed Settings

- **Forward Speed**: Start at 0.5, increase for faster following
- **Max Turn Speed**: Keep at 0.3-0.5 to prevent tip-over
- **Max Speed (DriveToPoint)**: Use 0.3-0.5 for precise positioning

## Important Notes

### Odometry Integration

This library requires an odometry system to track the robot's pose. You must:

1. Implement your odometry system (dead wheels, drive encoders + IMU, etc.)
2. Update the drive subsystem's pose in your OpMode's `run()` method:

```java
@Override
public void run() {
    // Update with your odometry
    Pose2D currentPose = odometry.getPose();
    drive.updatePose(currentPose);
    
    CommandScheduler.getInstance().run();
}
```

### Coordinate System

- **X-axis**: Right is positive
- **Y-axis**: Forward is positive
- **Heading**: Counter-clockwise is positive (standard math convention)
- **Units**: Inches for distance, radians for angles

### Motor Configuration

Ensure your motors are properly configured:
- Front right and back right typically need to be inverted
- Test with simple teleop before using autonomous commands

## Examples

See the `com.purepursuit.examples` package for complete examples:

- `ExampleAutonomous.java`: Complete autonomous with path following and command sequencing
- `ExampleTeleOp.java`: Field-centric teleop drive

## API Documentation

All classes include comprehensive Javadoc comments. Key features:

- Detailed class and method descriptions
- Parameter explanations
- Return value descriptions
- Usage examples
- Thread-safety notes

## Advanced Usage

### Custom Commands

Create your own commands using the subsystem:

```java
public class CustomCommand extends CommandBase {
    private final MecanumDriveSubsystem drive;
    
    public CustomCommand(MecanumDriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }
    
    @Override
    public void execute() {
        // Your custom logic here
        Pose2D pose = drive.getPose();
        // ... calculate control outputs
        drive.driveRobotCentric(strafe, forward, turn);
    }
    
    @Override
    public boolean isFinished() {
        return false; // Your completion condition
    }
}
```

### Multiple Subsystems

Combine the drive subsystem with other subsystems:

```java
public class IntakeSubsystem extends SubsystemBase {
    // Your intake code
}

// In OpMode
SequentialCommandGroup auto = new SequentialCommandGroup(
    new ParallelCommandGroup(
        new FollowPathCommand(drive, path),
        new IntakeCommand(intake)
    ),
    new ScoreCommand(drive, intake, scorer)
);
```

## Troubleshooting

### Robot doesn't follow path
- Check that pose is being updated correctly
- Verify motor directions are correct
- Tune lookahead distance (try increasing it)
- Check coordinate system orientation

### Robot oscillates
- Reduce lookahead distance
- Lower forward speed
- Tune heading PID (reduce P, increase D)

### Robot cuts corners
- Increase lookahead distance
- Add more waypoints
- Reduce forward speed on turns

### Commands don't run
- Ensure subsystem is registered: `register(drive)`
- Check that CommandScheduler.run() is called
- Verify command requirements don't conflict

## Contributing

Contributions are welcome! Please ensure:
- Code follows existing style
- All methods have Javadoc comments
- Examples are updated if API changes
- Changes are tested on actual hardware

## License

This library is released for FTC teams to use and modify as needed.

## Credits

Developed for the FIRST Tech Challenge community.

Based on:
- Pure Pursuit algorithm from robotics literature
- FTCLib command-based framework
- Standard PID control theory