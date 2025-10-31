// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/robot/RobotContainer.h"

using namespace frc::robot;

RobotContainer::RobotContainer() {
    // TODO: Initialize subsystems here

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
    // TODO: Configure button bindings and default commands here
}

std::optional<frc2::Command*> RobotContainer::getAutonomousCommand() {
    // TODO: Return the autonomous command
    return std::nullopt;
}

void RobotContainer::runSafetyInitialization() {
    // TODO: Run safety initialization
}
