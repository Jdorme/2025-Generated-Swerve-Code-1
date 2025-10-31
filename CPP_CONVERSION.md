# Java to C++ Conversion Guide

## Overview

This document tracks the conversion of the FRC 2025 robot code from Java to C++. The original Java codebase consists of approximately 8,000+ lines across 45 files implementing a complex swerve drive robot with multiple game piece manipulation subsystems.

## Conversion Status

### ✅ Completed

1. **Project Structure**
   - Created C++ directory structure (`src/main/cpp`, `src/main/include`)
   - Set up proper namespace hierarchy (`frc::robot::subsystems`, `frc::robot::commands`)
   - Created build.gradle.cpp for C++ builds

2. **Constants and Configuration**
   - `Constants.h` - All robot constants converted to C++ constexpr and namespaces
   - `TunerConstants.h/cpp` - Phoenix 6 swerve drive configuration

3. **Core Infrastructure**
   - `Main.cpp` - Entry point using `frc::StartRobot<>`
   - `Robot.h/cpp` - Main robot class with all state methods
   - `RobotContainer.h/cpp` - Command/subsystem container (stub implementation)

4. **Subsystems (Partial)**
   - `ArmSubsystem.h/cpp` - Arm control with Motion Magic
   - `EndgameLiftSubsystem.h/cpp` - Climb mechanism

### 🚧 In Progress / TODO

#### High Priority Subsystems

1. **CommandSwerveDrivetrain** (419 lines)
   - Complex Phoenix 6 swerve implementation
   - Requires conversion of telemetry and field-centric drive
   - Odometry and vision integration

2. **SafetySubsystem** (195 lines) - CRITICAL
   - Coordinates arm/elevator to prevent collisions
   - State machine logic for "danger zone" avoidance

3. **ElevatorSubsystem** (199 lines)
   - Dual-motor elevator with CANcoder feedback
   - Motion Magic control with gear ratio calculations

4. **AlgaeIntake** (291 lines)
   - Motor control with CANrange sensor
   - State machine for intake/hold/reverse

5. **CoralIntake** (225 lines)
   - TalonFXS motor control
   - CANrange distance sensor integration

6. **PhotonVisionSubsystem** (417 lines)
   - AprilTag detection and pose estimation
   - Camera management

#### Commands (30+ files)

**Coral Commands:**
- L1ScoreCommand
- L2ScoreCommand
- L3ScoreCommand
- L4ScoreCommand
- L4ScoreManualCommand
- StowedCommand
- PickupCommand
- etc.

**Algae Commands:**
- AlgaeNetCommand
- AlgaeFloorIntakeCommand
- AlgaeL2Command
- AlgaeL3Command
- AlgaeProcessorCommand

**Auto Alignment:**
- AutoAlignCommand
- LeftPoleAlignCommand
- RightPoleAlignCommand

**Utilities:**
- AutoCommandFactory
- SafeInitializationCommand

## Key Conversion Patterns

### Java → C++ Mappings

#### Imports and Includes
```cpp
// Java
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

// C++
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
```

#### Class Definitions
```cpp
// Java
public class ArmSubsystem extends SubsystemBase {
    private static final double GEAR_RATIO = 74.5;
    private final TalonFX motor;
}

// C++
class ArmSubsystem : public frc2::SubsystemBase {
private:
    static constexpr double GEAR_RATIO = 74.5;
    ctre::phoenix6::hardware::TalonFX motor;
}
```

#### Method Calls
```cpp
// Java
motor.getPosition().getValueAsDouble()

// C++
motor.GetPosition().GetValueAsDouble()
```

#### Configuration Objects
```cpp
// Java
var motorConfig = new TalonFXConfiguration();
motorConfig.Slot0 = slot0Configs;

// C++
TalonFXConfiguration motorConfig{};
motorConfig.Slot0 = slot0Configs;
```

#### Units Library
```cpp
// Java
MetersPerSecond.of(5.21)
Inches.of(2)
Rotations.of(0.407470703125)

// C++
5.21_mps
2.0_in
0.407470703125_tr
```

### Common Patterns

1. **SmartDashboard**
   ```cpp
   // Java
   SmartDashboard.putNumber("key", value);

   // C++
   frc::SmartDashboard::PutNumber("key", value);
   ```

2. **Command Scheduling**
   ```cpp
   // Java
   CommandScheduler.getInstance().run();

   // C++
   frc2::CommandScheduler::GetInstance().Run();
   ```

3. **Optional Values**
   ```cpp
   // Java
   Command m_autonomousCommand;
   if (m_autonomousCommand != null) {
       m_autonomousCommand.schedule();
   }

   // C++
   std::optional<frc2::Command*> m_autonomousCommand;
   if (m_autonomousCommand.has_value() && m_autonomousCommand.value() != nullptr) {
       m_autonomousCommand.value()->Schedule();
   }
   ```

## Building the C++ Project

### Prerequisites
- WPILib 2025.3.2 or later
- GradleRIO plugin
- Vendor dependencies (already in `vendordeps/`):
  - Phoenix6-frc2025-latest.json
  - PathplannerLib-2025.2.7.json
  - photonlib.json

### Build Commands

1. **Switch to C++ build system:**
   ```bash
   mv build.gradle build.gradle.java
   mv build.gradle.cpp build.gradle
   ```

2. **Build:**
   ```bash
   ./gradlew build
   ```

3. **Deploy to robot:**
   ```bash
   ./gradlew deploy
   ```

4. **Run simulation:**
   ```bash
   ./gradlew simulateNative
   ```

## Remaining Work Estimate

| Component | Files | Estimated Lines | Priority |
|-----------|-------|-----------------|----------|
| CommandSwerveDrivetrain | 1 | 419 | CRITICAL |
| SafetySubsystem | 1 | 195 | CRITICAL |
| ElevatorSubsystem | 1 | 199 | HIGH |
| AlgaeIntake | 1 | 291 | HIGH |
| CoralIntake | 1 | 225 | HIGH |
| PhotonVisionSubsystem | 1 | 417 | HIGH |
| LimelightHelpers | 1 | ~500 | MEDIUM |
| Coral Commands | 8 | ~400 | HIGH |
| Algae Commands | 7 | ~350 | HIGH |
| Auto Commands | 3 | ~200 | MEDIUM |
| RobotContainer (full) | 1 | ~200 | HIGH |
| Telemetry | 1 | ~100 | LOW |
| **TOTAL** | **26** | **~3,500** | |

## Testing Strategy

1. **Compilation Testing**
   - Ensure all files compile without errors
   - Resolve any Phoenix 6 C++ API differences

2. **Subsystem Testing**
   - Test each subsystem individually
   - Verify motor control and sensor readings
   - Check Motion Magic profiles

3. **Safety Testing**
   - Verify SafetySubsystem prevents collisions
   - Test all position setpoints
   - Ensure soft limits work correctly

4. **Integration Testing**
   - Test command sequences
   - Verify autonomous routines
   - Test driver controls

## Notes

- Phoenix 6 C++ API is similar but not identical to Java (different casing)
- Vendor dependencies support both Java and C++
- PathPlanner C++ library is available
- PhotonVision has C++ support

## Resources

- [WPILib C++ Documentation](https://docs.wpilib.org/en/stable/docs/software/cpp-guide/index.html)
- [Phoenix 6 C++ API](https://v6.docs.ctr-electronics.com/en/stable/)
- [PathPlanner C++ Documentation](https://pathplanner.dev/home.html)
- [PhotonVision C++ Documentation](https://docs.photonvision.org/)
