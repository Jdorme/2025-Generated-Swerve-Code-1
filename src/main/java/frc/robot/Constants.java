// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

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
    public static final int elevatorEncoderID = 40;

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
                public static final double[] STOWED = {4, 0.0};
                public static final double[] L4 = {25.5, 67};
                public static final double[] L3 = {9, 34};
                public static final double[] L2 = {2.5, 45};
                public static final double[] L1 = {2.5, -130};
                public static final double[] PICKUP = {8.5, -115};
                
                public static final double[] L3_ALGAE = {9, -120};
                public static final double[] L2_ALGAE = {0, -120};
                public static final double[] GROUND_ALGAE = {8, -75};
                public static final double[] PROCESSOR_ALGAE = {0, -60};
                public static final double[] NET_ALGAE = {27.5, -185};

                public static final double[] CLIMB_POSITION = {0, 90};


                public static final double[] Start_Position = {0, 0};
}

public static final class EndgameLiftConstants {
    public static final int liftMotorTopID = 20;
    public static final int liftMotorBottomID = 21;

    // Motor configuration
    public static final double CURRENT_LIMIT = 40.0; // Amps
    
    // Control constants
    public static final double LIFT_UP_SPEED = 1; // 100% speed up
    public static final double LIFT_DOWN_SPEED = -1; // 100% speed down
                                                                                              
    // PID Constants
    public static final class PIDConstants {
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.02;
    }
    
    // Position Constants - Note: The naming is reversed from actual movement direction
    // POSITION_RETRACTED actually causes the lift to extend outward from the robot (negative motor rotation)
    // POSITION_EXTENDED actually causes the lift to retract into the robot (positive motor rotation)
    public static final double POSITION_RETRACTED = -75; // Outward position (near reverse soft limit -82)
    public static final double POSITION_EXTENDED = 80; // Inward position (near forward soft limit 85)
}

// Add this to your existing Constants.java file

public static final class ReefAlignmentConstants {
    // Camera Selection - update these with your actual camera IDs
    public static final String LEFT_POLE_CAMERA = "ElevatorCam";  // Camera for left pole alignment
    public static final String RIGHT_POLE_CAMERA = "EndGameCam";  // Camera for right pole alignment
    
    // Tag Alignment Configuration
    public static final int[] VALID_REEF_TAG_IDS_BLUE = {17, 18, 19, 20, 21, 22};
    public static final int[] VALID_REEF_TAG_IDS_RED = {6, 7, 8, 9, 10, 11};
    
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
    public static final boolean TESTING_MODE = false;  // Set to false for competition

    public static final class ElevatorMotionMagic {
        // Testing speeds (slower)
        private static final double TEST_CRUISE_VELOCITY = 15;  // 25% of competition speed
        private static final double TEST_ACCELERATION = 25;     // 25% of competition speed
        private static final double TEST_JERK = 200;           // 25% of competition speed

        // Competition speeds (faster)
        private static final double COMP_CRUISE_VELOCITY = 160;//120
        private static final double COMP_ACCELERATION = 120;//80
        private static final double COMP_JERK = 1900;

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
        private static final double COMP_CRUISE_VELOCITY = 250;
        private static final double COMP_ACCELERATION = 180;
        private static final double COMP_JERK = 1900;

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
