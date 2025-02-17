// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.IntegratedMechanismSubsystem.Position;

/** Add your docs here. */
public class Constants {

public static final class AlgaeIntakeConstants {

    public static final int algaeIntakeMotorID = 33;
    public static final int algaeIntakeCANrangeID = 37;

}
    
public static final class CoralIntakeConstants {

    public static final int coralIntakeMotorID = 35; 
    public static final int coralIntakeCANrangeID = 36;

}

public static final class ElevatorConstants {

    public static final int elevatorMotorRID = 31;
    public static final int elevatorMotorLID = 32;

    public static final int encoder1DIO = 3; //??????

    //------------Setpoints---------------
    public static final double BOTTOM_POSITION = 0.0;
    public static final double MIDDLE_POSITION = 11.0;
    public static final double TOP_POSITION = 22.28;

}
public static final class ArmConstants {
    public static final int armMotorID = 17;
    
}

public static final class SafetyConstants{
        //------------Setpoints---------------
                // (elevator height, arm angle)
                public static final double[] STOWED = {11, 0.0};
                public static final double[] L4 = {22.28, 25};
                public static final double[] L3 = {16.5, 25};
                public static final double[] L2 = {11.0, 25};
                public static final double[] PICKUP = {14.5, -125};
                public static final double[] Start_Position = {0, 0};
}

public static final class ReefAlignmentConstants {
    // Tag Alignment Configuration
    public static final int[] VALID_REEF_TAG_IDS = {6, 7, 8, 9, 10, 11};
    
    // Alignment Distance Parameters
    public static final double TARGET_DISTANCE = 0.6; // meters from tag
    
    // Tolerance Parameters
    public static final double ALIGNMENT_TOLERANCE_METERS = 0.05; 
    public static final double ROTATION_TOLERANCE_DEGREES = 2.0;
    
    // Retry Mechanism
    public static final int MAX_DETECTION_ATTEMPTS = 3;
    public static final double DETECTION_RETRY_DELAY_SECONDS = 0.5;
    public static final double ALIGNMENT_TIMEOUT_SECONDS = 10.0;
    
    // PID Configuration for X Alignment
    public static final class XAxisPID {
        public static final double kP = 1.5;
        public static final double kI = 0.0;
        public static final double kD = 0.1;
    }
    
    // PID Configuration for Y Alignment
    public static final class YAxisPID {
        public static final double kP = 1.5;
        public static final double kI = 0.0;
        public static final double kD = 0.1;
    }
    
    // PID Configuration for Rotation Alignment
    public static final class RotationPID {
        public static final double kP = 0.05;
        public static final double kI = 0.0;
        public static final double kD = 0.005;
    }
}

// Add to Constants.java

public static final class MotionMagicConstants {
    // Toggle between testing and competition speeds
    public static final boolean TESTING_MODE = true;  // Set to false for competition

    public static final class ElevatorMotionMagic {
        // Testing speeds (slower)
        private static final double TEST_CRUISE_VELOCITY = 15;  // 25% of competition speed
        private static final double TEST_ACCELERATION = 25;     // 25% of competition speed
        private static final double TEST_JERK = 200;           // 25% of competition speed

        // Competition speeds (faster)
        private static final double COMP_CRUISE_VELOCITY = 120;
        private static final double COMP_ACCELERATION = 200;
        private static final double COMP_JERK = 1600;

        // Getter methods that return the appropriate speed based on mode
        public static double getCruiseVelocity() {
            return TESTING_MODE ? TEST_CRUISE_VELOCITY : COMP_CRUISE_VELOCITY;
        }

        public static double getAcceleration() {
            return TESTING_MODE ? TEST_ACCELERATION : COMP_ACCELERATION;
        }

        public static double getJerk() {
            return TESTING_MODE ? TEST_JERK : COMP_JERK;
        }
    }

    public static final class ArmMotionMagic {
        // Testing speeds (slower)
        private static final double TEST_CRUISE_VELOCITY = 20;  // Even slower for arm testing
        private static final double TEST_ACCELERATION = 40;     
        private static final double TEST_JERK = 300;           

        // Competition speeds (faster)
        private static final double COMP_CRUISE_VELOCITY = 120;
        private static final double COMP_ACCELERATION = 200;
        private static final double COMP_JERK = 1600;

        // Getter methods that return the appropriate speed based on mode
        public static double getCruiseVelocity() {
            return TESTING_MODE ? TEST_CRUISE_VELOCITY : COMP_CRUISE_VELOCITY;
        }

        public static double getAcceleration() {
            return TESTING_MODE ? TEST_ACCELERATION : COMP_ACCELERATION;
        }

        public static double getJerk() {
            return TESTING_MODE ? TEST_JERK : COMP_JERK;
        }
    }
}

//Drivetrain IDs--------------------------------------------------

//Summary (don't use): 0-11

    // Front Left
    // private static final int kFrontLeftDriveMotorId = 0;
    // private static final int kFrontLeftSteerMotorId = 3;
    // private static final int kFrontLeftEncoderId = 10;

    // // Front Right
    // private static final int kFrontRightDriveMotorId = 5;
    // private static final int kFrontRightSteerMotorId = 6;
    // private static final int kFrontRightEncoderId = 9;

    // // Back Left
    // private static final int kBackLeftDriveMotorId = 7;
    // private static final int kBackLeftSteerMotorId = 4;
    // private static final int kBackLeftEncoderId = 11;

    // // Back Right
    // private static final int kBackRightDriveMotorId = 1;
    // private static final int kBackRightSteerMotorId = 2;
    // private static final int kBackRightEncoderId = 8;

//-----------------------------------------------------------------

}
