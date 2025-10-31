// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/robot/subsystems/ArmSubsystem.h"
#include "frc/robot/Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <algorithm>
#include <cmath>

using namespace frc::robot::subsystems;
using namespace ctre::phoenix6;
using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::signals;

ArmSubsystem::ArmSubsystem()
    : motor(Constants::ArmConstants::armMotorID) {
    configureMotor();
}

void ArmSubsystem::configureMotor() {
    TalonFXConfiguration motorConfig{};

    // Basic motor configuration
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue::Brake;

    // Current limits for safety
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Configure the PID gains
    Slot0Configs slot0Configs{};
    slot0Configs.kP = 15.0;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.1;
    slot0Configs.kV = 0.12;
    motorConfig.Slot0 = slot0Configs;

    // Motion Magic settings
    MotionMagicConfigs motionMagicConfigs{};
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants::MotionMagicConstants::ArmMotionMagic::getCruiseVelocity();
    motionMagicConfigs.MotionMagicAcceleration = Constants::MotionMagicConstants::ArmMotionMagic::getAcceleration();
    motionMagicConfigs.MotionMagicJerk = Constants::MotionMagicConstants::ArmMotionMagic::getJerk();
    motorConfig.MotionMagic = motionMagicConfigs;

    // Configure motor inversion
    motorConfig.MotorOutput.Inverted = InvertedValue::CounterClockwise_Positive;
    motor.GetConfigurator().Apply(motorConfig);
}

void ArmSubsystem::setAngle(double angleDegrees) {
    // Clamp angle to valid range between MIN_ANGLE and MAX_ANGLE
    targetPositionDegrees = std::clamp(angleDegrees, MIN_ANGLE, MAX_ANGLE);

    // Convert degrees to motor rotations
    double motorRotations = (targetPositionDegrees / 360.0) * GEAR_RATIO;

    motor.SetControl(motionRequest.WithPosition(motorRotations));
}

double ArmSubsystem::getCurrentAngle() {
    // Get motor rotations and convert to degrees
    double motorRotations = motor.GetPosition().GetValueAsDouble();
    return (motorRotations / GEAR_RATIO) * 360.0;
}

bool ArmSubsystem::isAtTarget() {
    return std::abs(getErrorDegrees()) < 2.0;
}

double ArmSubsystem::getErrorDegrees() {
    // Get the closed-loop error in rotations
    double errorRotations = motor.GetClosedLoopError().GetValueAsDouble();
    return (errorRotations / GEAR_RATIO) * 360.0;
}

void ArmSubsystem::stop() {
    motor.StopMotor();
}

void ArmSubsystem::setMotorOutput(double percentOutput) {
    motor.Set(percentOutput);
}

void ArmSubsystem::Periodic() {
    // Log arm information to SmartDashboard
    frc::SmartDashboard::PutNumber("Arm/Current Angle", getCurrentAngle());
    frc::SmartDashboard::PutNumber("Arm/Target Angle", targetPositionDegrees);
    frc::SmartDashboard::PutNumber("Arm/Position Error", getErrorDegrees());
    frc::SmartDashboard::PutBoolean("Arm/At Target", isAtTarget());
}
