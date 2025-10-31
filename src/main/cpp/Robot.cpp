// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/robot/Robot.h"
#include "frc/robot/RobotContainer.h"
#include <frc2/command/CommandScheduler.h>

using namespace frc::robot;

Robot::Robot() {
    m_robotContainer = std::make_unique<RobotContainer>();
}

void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    m_autonomousCommand = m_robotContainer->getAutonomousCommand();

    if (m_autonomousCommand.has_value() && m_autonomousCommand.value() != nullptr) {
        m_autonomousCommand.value()->Schedule();
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    if (m_autonomousCommand.has_value() && m_autonomousCommand.value() != nullptr) {
        m_autonomousCommand.value()->Cancel();
    }
    m_robotContainer->runSafetyInitialization();
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

void Robot::SimulationPeriodic() {}
