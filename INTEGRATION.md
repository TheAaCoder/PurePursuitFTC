# Integration with FTC Projects

This document explains how to integrate the PurePursuitFTC library into your actual FTC robot project.

## For Real FTC Projects

When using this library in an actual FTC project (with the FTC SDK), you need to:

### 1. Add FTCLib Dependency

Add to your TeamCode module's `build.gradle`:

```gradle
dependencies {
    implementation 'com.arcrobotics:ftclib:2.1.1'
    // Your other dependencies...
}
```

### 2. Copy the Library Files

Copy the `com.purepursuit` package (excluding examples if you want) to your TeamCode:

```
TeamCode/
  src/
    main/
      java/
        org/
          firstinspires/
            ftc/
              teamcode/
                com/
                  purepursuit/     <-- Copy this entire folder
                    commands/
                    control/
                    geometry/
                    pathing/
                    subsystems/
                    examples/     <-- Optional, for reference
```

### 3. Adapt the Examples

The example OpModes need to extend FTCLib's `CommandOpMode`:

```java
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "My Auto")
public class MyAuto extends CommandOpMode {
    // Your code here
}
```

### 4. Required Components

Your robot needs:

1. **Drive Motors**: Configured in your robot configuration
2. **Odometry System**: To track robot pose (dead wheels, drive encoders + IMU, etc.)
3. **FTC SDK**: Version 8.0 or later
4. **FTCLib**: Version 2.1.1 or later

## Why FTCLib?

FTCLib provides:
- **Command-based framework**: Organize code into reusable commands and subsystems
- **Motor wrappers**: Simplified motor control
- **Gamepad enhancements**: Better input handling
- **Utilities**: Drive classes, PID controllers (note: this library provides its own PID for path following)

## Standalone Building (Development)

For testing the core library logic without FTC SDK/FTCLib dependencies:

```bash
# This builds only the core geometry, control, and pathing classes
gradle build
```

The standalone build excludes:
- `com.purepursuit.subsystems.*` (depends on FTCLib)
- `com.purepursuit.commands.*` (depends on FTCLib)
- `com.purepursuit.examples.*` (depends on FTC SDK and FTCLib)

These classes will be available when you copy the library into a full FTC project.

## Example Robot Configuration

In your FTC robot configuration (via Driver Station or Robot Controller):

```
Control Hub Portal:
  Motors:
    - frontLeft (port 0)
    - frontRight (port 1)
    - backLeft (port 2)
    - backRight (port 3)
  
  IMU:
    - imu (built-in)
    
  Encoders (if using dead wheels):
    - leftEncoder (port ...)
    - rightEncoder (port ...)
    - centerEncoder (port ...)
```

## Odometry Integration

The library requires pose updates. Here's a minimal example:

```java
@Autonomous(name = "Pure Pursuit Auto")
public class PurePursuitAuto extends CommandOpMode {
    private MecanumDriveSubsystem drive;
    private YourOdometryClass odometry;
    
    @Override
    public void initialize() {
        // Setup hardware
        Motor fl = new Motor(hardwareMap, "frontLeft");
        Motor fr = new Motor(hardwareMap, "frontRight");
        Motor bl = new Motor(hardwareMap, "backLeft");
        Motor br = new Motor(hardwareMap, "backRight");
        
        drive = new MecanumDriveSubsystem(fl, fr, bl, br);
        odometry = new YourOdometryClass(hardwareMap);
        
        // Create and schedule commands
        Path path = new Path()
            .addWaypoint(0, 0, 12.0)
            .addWaypoint(24, 24, 12.0);
        
        schedule(new FollowPathCommand(drive, path));
        register(drive);
    }
    
    @Override
    public void run() {
        // Update odometry
        odometry.update();
        
        // Update drive subsystem with current pose
        drive.updatePose(odometry.getPose());
        
        // Run command scheduler
        super.run();
        
        // Telemetry
        telemetry.addData("X", drive.getPose().x);
        telemetry.addData("Y", drive.getPose().y);
        telemetry.update();
    }
}
```

## Common Integration Issues

### Issue: "Cannot resolve FTCLib classes"

**Solution**: Add FTCLib to your build.gradle and sync the project

### Issue: "Cannot resolve FTC SDK classes"

**Solution**: Ensure you're in a proper FTC project with the SDK. The examples require:
- `com.qualcomm.robotcore.eventloop.opmode.*`
- `com.qualcomm.robotcore.hardware.*`

### Issue: Commands don't run

**Solution**: 
1. Ensure you're calling `super.run()` in your run() method
2. Register subsystems with `register(subsystem)`
3. Schedule commands with `schedule(command)`

### Issue: Robot doesn't move/behaves incorrectly

**Solution**:
1. Verify motor directions (invert as needed)
2. Check odometry is updating correctly
3. Verify coordinate system (X=right, Y=forward)
4. Test with simple teleop first

## Testing Your Integration

1. **Test Basic Drive**:
   ```java
   drive.driveRobotCentric(0, 0.5, 0); // Drive forward
   ```

2. **Test Odometry**:
   - Move robot manually
   - Check pose updates in telemetry

3. **Test Simple Path**:
   - Create a straight line path
   - Test with slow speed first

4. **Test Complex Paths**:
   - Add turns and waypoints
   - Tune lookahead and speeds

## Compatibility

- **FTC SDK**: 8.0+
- **FTCLib**: 2.1.1+
- **Android**: SDK level 24+
- **Java**: 8+

## Additional Resources

- [FTCLib Docs](https://docs.ftclib.org/)
- [FTC SDK GitHub](https://github.com/FIRST-Tech-Challenge/FtcRobotController)
- [Usage Guide](USAGE_GUIDE.md)
- [README](README.md)
