// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>

namespace frc {
namespace robot {
namespace subsystems {

class EndgameLiftSubsystem : public frc2::SubsystemBase {
public:
    EndgameLiftSubsystem();

    void liftUp();
    void liftDown();
    void stop();

    void Periodic() override;

private:
    // Motors
    ctre::phoenix6::hardware::TalonFX topMotor;
    ctre::phoenix6::hardware::TalonFX bottomMotor;

    // Control request
    ctre::phoenix6::controls::DutyCycleOut dutyCycleControl{0};

    void configureMotors();
    void setSpeed(double speed);
};

} // namespace subsystems
} // namespace robot
} // namespace frc
