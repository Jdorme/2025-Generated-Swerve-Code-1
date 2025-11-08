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

public static final class ReefAlignConstants {
    // PID Controllers
    /*
    P (Proportional)** - How aggressively to correct errors
    - Too LOW: Robot moves slowly, takes forever to align
    - Too HIGH: Robot oscillates/shakes back and forth
    - Start at 2.0 for translation, 0.05 for rotation
    */
    public static final double X_REEF_ALIGNMENT_P = 0.25;
    public static final double Y_REEF_ALIGNMENT_P = 0.25;
    public static final double ROT_REEF_ALIGNMENT_P = 0.05;
    
    // Setpoints
    public static final double X_SETPOINT_REEF_ALIGNMENT = -.2;// meters from tag
    public static final double rightY_SETPOINT_REEF_ALIGNMENT = 0.2;// meters to the side
    public static final double leftY_SETPOINT_REEF_ALIGNMENT = 0;// meters to the side
    public static final double ROT_SETPOINT_REEF_ALIGNMENT = -31;// face tag directly
    
    // Tolerances
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.05;// 5cm
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.05;// 5cm
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 3.0;// 3 degrees
    
    // Timing
    public static final double DONT_SEE_TAG_WAIT_TIME = 2.0;// seconds
    public static final double POSE_VALIDATION_TIME = 0.3;// seconds
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
