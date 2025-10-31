// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/robot/generated/TunerConstants.h"
#include "frc/robot/subsystems/CommandSwerveDrivetrain.h"

using namespace generated;
using namespace ctre::phoenix6;
using namespace ctre::phoenix6::swerve;
using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::signals;

void TunerConstants::Initialize() {
    // Initialize steer gains
    steerGains.kP = 100.0;
    steerGains.kI = 0.0;
    steerGains.kD = 0.5;
    steerGains.kS = 0.1;
    steerGains.kV = 1.59;
    steerGains.kA = 0.0;
    steerGains.StaticFeedforwardSign = StaticFeedforwardSignValue::UseClosedLoopSign;

    // Initialize drive gains
    driveGains.kP = 0.1;
    driveGains.kI = 0.0;
    driveGains.kD = 0.0;
    driveGains.kS = 0.0;
    driveGains.kV = 0.124;

    // Initialize steer initial configs with current limits
    CurrentLimitsConfigs currentLimits{};
    currentLimits.StatorCurrentLimit = 60.0;
    currentLimits.StatorCurrentLimitEnable = true;
    steerInitialConfigs.CurrentLimits = currentLimits;

    // Initialize drivetrain constants
    DrivetrainConstants
        .WithCANBusName(kCANBus.GetName())
        .WithPigeon2Id(kPigeonId)
        .WithPigeon2Configs(pigeonConfigs);

    // Initialize module constants factory
    ConstantCreator
        .WithDriveMotorGearRatio(kDriveGearRatio)
        .WithSteerMotorGearRatio(kSteerGearRatio)
        .WithCouplingGearRatio(kCoupleRatio)
        .WithWheelRadius(kWheelRadius)
        .WithSteerMotorGains(steerGains)
        .WithDriveMotorGains(driveGains)
        .WithSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
        .WithDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
        .WithSlipCurrent(kSlipCurrent)
        .WithSpeedAt12Volts(kSpeedAt12Volts)
        .WithDriveMotorType(kDriveMotorType)
        .WithSteerMotorType(kSteerMotorType)
        .WithFeedbackSource(kSteerFeedbackType)
        .WithDriveMotorInitialConfigs(driveInitialConfigs)
        .WithSteerMotorInitialConfigs(steerInitialConfigs)
        .WithEncoderInitialConfigs(encoderInitialConfigs)
        .WithSteerInertia(kSteerInertia)
        .WithDriveInertia(kDriveInertia)
        .WithSteerFrictionVoltage(kSteerFrictionVoltage)
        .WithDriveFrictionVoltage(kDriveFrictionVoltage);

    // Create module constants
    FrontLeft = ConstantCreator.CreateModuleConstants(
        kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
        kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
    );

    FrontRight = ConstantCreator.CreateModuleConstants(
        kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
        kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
    );

    BackLeft = ConstantCreator.CreateModuleConstants(
        kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
        kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
    );

    BackRight = ConstantCreator.CreateModuleConstants(
        kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
        kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
    );
}

CommandSwerveDrivetrain* TunerConstants::createDrivetrain() {
    return new CommandSwerveDrivetrain(
        DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
    );
}
