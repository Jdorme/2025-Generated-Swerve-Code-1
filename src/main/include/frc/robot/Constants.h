// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/angle.h>
#include <units/length.h>
#include <array>

namespace Constants {

namespace AlgaeIntakeConstants {
    constexpr int algaeIntakeMotorID = 33;
    constexpr int algaeIntakeCANrangeID = 37;
}

namespace CoralIntakeConstants {
    constexpr int coralIntakeMotorID = 35;
    constexpr int coralIntakeCANrangeID = 36;
}

namespace ElevatorConstants {
    constexpr int elevatorMotorRID = 31;
    constexpr int elevatorMotorLID = 32;
    constexpr int elevatorEncoderID = 40;
    constexpr int encoder1DIO = 3;

    // Setpoints
    constexpr double BOTTOM_POSITION = 0.0;
    constexpr double MIDDLE_POSITION = 11.0;
    constexpr double TOP_POSITION = 22.28;
}

namespace ArmConstants {
    constexpr int armMotorID = 17;
}

namespace SafetyConstants {
    // Setpoints (elevator height, arm angle)
    constexpr std::array<double, 2> STOWED = {4, 0.0};
    constexpr std::array<double, 2> L4 = {25.5, 67};
    constexpr std::array<double, 2> L3 = {9, 34};
    constexpr std::array<double, 2> L2 = {2.5, 45};
    constexpr std::array<double, 2> L1 = {2.5, -130};
    constexpr std::array<double, 2> PICKUP = {8.5, -115};

    constexpr std::array<double, 2> L3_ALGAE = {9, -120};
    constexpr std::array<double, 2> L2_ALGAE = {0, -120};
    constexpr std::array<double, 2> GROUND_ALGAE = {8, -75};
    constexpr std::array<double, 2> PROCESSOR_ALGAE = {0, -60};
    constexpr std::array<double, 2> NET_ALGAE = {27.5, -185};

    constexpr std::array<double, 2> CLIMB_POSITION = {0, 90};
    constexpr std::array<double, 2> Start_Position = {0, 0};
}

namespace EndgameLiftConstants {
    constexpr int liftMotorTopID = 20;
    constexpr int liftMotorBottomID = 21;

    // Motor configuration
    constexpr double CURRENT_LIMIT = 40.0; // Amps

    // Control constants
    constexpr double LIFT_UP_SPEED = 1.0;   // 100% speed up
    constexpr double LIFT_DOWN_SPEED = -1.0; // 100% speed down

    namespace PIDConstants {
        constexpr double kP = 0.1;
        constexpr double kI = 0.0;
        constexpr double kD = 0.0;
        constexpr double kV = 0.02;
    }

    // Position Constants
    constexpr double POSITION_RETRACTED = -75.0; // Outward position
    constexpr double POSITION_EXTENDED = 80.0;   // Inward position
}

namespace ReefAlignmentConstants {
    // Camera Selection
    constexpr const char* LEFT_POLE_CAMERA = "ElevatorCam";
    constexpr const char* RIGHT_POLE_CAMERA = "EndGameCam";

    // Tag Alignment Configuration
    constexpr std::array<int, 6> VALID_REEF_TAG_IDS_BLUE = {17, 18, 19, 20, 21, 22};
    constexpr std::array<int, 6> VALID_REEF_TAG_IDS_RED = {6, 7, 8, 9, 10, 11};

    // Alignment Distance Parameters
    constexpr double TARGET_DISTANCE = 0.6; // meters from tag

    // Tolerance Parameters
    constexpr double ALIGNMENT_TOLERANCE_METERS = 0.05;
    constexpr double ROTATION_TOLERANCE_DEGREES = 2.0;

    // Retry Mechanism
    constexpr int MAX_DETECTION_ATTEMPTS = 3;
    constexpr double DETECTION_RETRY_DELAY_SECONDS = 0.5;
    constexpr double ALIGNMENT_TIMEOUT_SECONDS = 10.0;

    // PID Configuration for X Alignment
    namespace XAxisPID {
        constexpr double kP = 1.5;
        constexpr double kI = 0.0;
        constexpr double kD = 0.1;
    }

    // PID Configuration for Y Alignment
    namespace YAxisPID {
        constexpr double kP = 1.5;
        constexpr double kI = 0.0;
        constexpr double kD = 0.1;
    }

    // PID Configuration for Rotation Alignment
    namespace RotationPID {
        constexpr double kP = 0.05;
        constexpr double kI = 0.0;
        constexpr double kD = 0.005;
    }
}

namespace MotionMagicConstants {
    // Toggle between testing and competition speeds
    constexpr bool TESTING_MODE = false;  // Set to false for competition

    namespace ElevatorMotionMagic {
        // Testing speeds (slower)
        constexpr double TEST_CRUISE_VELOCITY = 15.0;
        constexpr double TEST_ACCELERATION = 25.0;
        constexpr double TEST_JERK = 200.0;

        // Competition speeds (faster)
        constexpr double COMP_CRUISE_VELOCITY = 160.0;
        constexpr double COMP_ACCELERATION = 120.0;
        constexpr double COMP_JERK = 1900.0;

        // Getter functions that return the appropriate speed based on mode
        constexpr double getCruiseVelocity() {
            return TESTING_MODE ? TEST_CRUISE_VELOCITY : COMP_CRUISE_VELOCITY;
        }

        constexpr double getAcceleration() {
            return TESTING_MODE ? TEST_ACCELERATION : COMP_ACCELERATION;
        }

        constexpr double getJerk() {
            return TESTING_MODE ? TEST_JERK : COMP_JERK;
        }
    }

    namespace ArmMotionMagic {
        // Testing speeds (slower)
        constexpr double TEST_CRUISE_VELOCITY = 20.0;
        constexpr double TEST_ACCELERATION = 40.0;
        constexpr double TEST_JERK = 300.0;

        // Competition speeds (faster)
        constexpr double COMP_CRUISE_VELOCITY = 250.0;
        constexpr double COMP_ACCELERATION = 180.0;
        constexpr double COMP_JERK = 1900.0;

        // Getter functions that return the appropriate speed based on mode
        constexpr double getCruiseVelocity() {
            return TESTING_MODE ? TEST_CRUISE_VELOCITY : COMP_CRUISE_VELOCITY;
        }

        constexpr double getAcceleration() {
            return TESTING_MODE ? TEST_ACCELERATION : COMP_ACCELERATION;
        }

        constexpr double getJerk() {
            return TESTING_MODE ? TEST_JERK : COMP_JERK;
        }
    }
}

} // namespace Constants
