// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>

namespace frc {
namespace robot {
namespace subsystems {

class ArmSubsystem : public frc2::SubsystemBase {
public:
    ArmSubsystem();

    void setAngle(double angleDegrees);
    double getCurrentAngle();
    bool isAtTarget();
    double getErrorDegrees();
    void stop();
    void setMotorOutput(double percentOutput);

    void Periodic() override;

private:
    // Physical constants
    static constexpr double GEAR_RATIO = 74.5; // 74.5:1 reduction (16t sprocket to 22t)
    static constexpr double MAX_ANGLE = 180.0; // degrees
    static constexpr double MIN_ANGLE = -210.0; // degrees

    // Motor
    ctre::phoenix6::hardware::TalonFX motor;
    ctre::phoenix6::controls::MotionMagicVoltage motionRequest{0};

    // Target position tracking
    double targetPositionDegrees = 0.0;

    void configureMotor();
};

} // namespace subsystems
} // namespace robot
} // namespace frc
