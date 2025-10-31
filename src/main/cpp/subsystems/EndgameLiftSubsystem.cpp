// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/robot/subsystems/EndgameLiftSubsystem.h"
#include "frc/robot/Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

using namespace frc::robot::subsystems;
using namespace ctre::phoenix6;
using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::signals;
using namespace Constants::EndgameLiftConstants;

EndgameLiftSubsystem::EndgameLiftSubsystem()
    : topMotor(liftMotorTopID),
      bottomMotor(liftMotorBottomID) {
    configureMotors();
}

void EndgameLiftSubsystem::configureMotors() {
    TalonFXConfiguration motorConfig{};

    // Basic motor configuration
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue::Brake;

    // Current limits for safety
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;

    // Configure soft limits
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 74.0;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -81.0;

    // Configure top motor
    motorConfig.MotorOutput.Inverted = InvertedValue::CounterClockwise_Positive;
    topMotor.GetConfigurator().Apply(motorConfig);

    // Configure bottom motor (inverted)
    motorConfig.MotorOutput.Inverted = InvertedValue::Clockwise_Positive;
    bottomMotor.GetConfigurator().Apply(motorConfig);
}

void EndgameLiftSubsystem::liftUp() {
    setSpeed(LIFT_UP_SPEED);
}

void EndgameLiftSubsystem::liftDown() {
    setSpeed(LIFT_DOWN_SPEED);
}

void EndgameLiftSubsystem::stop() {
    setSpeed(0.0);
}

void EndgameLiftSubsystem::setSpeed(double speed) {
    topMotor.SetControl(dutyCycleControl.WithOutput(speed));
    bottomMotor.SetControl(dutyCycleControl.WithOutput(speed));
}

void EndgameLiftSubsystem::Periodic() {
    // Update dashboard with motor currents for monitoring
    frc::SmartDashboard::PutNumber("EndgameLift/Top Current",
        topMotor.GetSupplyCurrent().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("EndgameLift/Bottom Current",
        bottomMotor.GetSupplyCurrent().GetValueAsDouble());
}
