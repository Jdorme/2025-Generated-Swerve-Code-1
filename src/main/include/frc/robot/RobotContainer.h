// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <optional>

namespace frc {
namespace robot {

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
public:
    RobotContainer();

    std::optional<frc2::Command*> getAutonomousCommand();
    void runSafetyInitialization();

private:
    // TODO: Add subsystems and commands here
    // Example:
    // std::unique_ptr<CommandSwerveDrivetrain> m_drivetrain;
    // std::unique_ptr<ArmSubsystem> m_arm;
    // etc.

    void ConfigureBindings();
};

} // namespace robot
} // namespace frc
