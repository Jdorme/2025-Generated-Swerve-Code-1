package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntegratedMechanismSubsystem extends SubsystemBase {
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    
    // Safety threshold
    private static final double ELEVATOR_SAFE_HEIGHT = 10.0; // inches
    private static final double ARM_SAFE_ANGLE = 0.0; // degrees
    private static final double ANGLE_TOLERANCE = 1.0; // degrees
    
    // Define move states
    private enum MoveState {
        MOVING_TO_SAFE_HEIGHT,  // Moving elevator up to safe height before moving arm
        MOVING_ARM,             // Moving arm once elevator is at safe height
        WAITING_FOR_ARM_ZERO,   // Waiting for arm to reach 0 before lowering elevator
        MOVING_ELEVATOR,        // Moving elevator to final height
        MOVING_TOGETHER,        // Moving both simultaneously (only when safe)
        DONE                    // Movement complete
    }
    
    // Preset positions remain the same
    public static class Positions {
        public static final Position STOWED = new Position(11, 0.0);
        public static final Position L4 = new Position(22.28, 65);
        public static final Position L3 = new Position(16.5, 75);
        public static final Position L2 = new Position(11.0, 90);
        public static final Position PICKUP = new Position(12.0, -115);
        public static final Position Start_Position = new Position(0, 0);
    }
    
    public static class Position {
        public final double elevatorHeight;
        public final double armAngle;
        
        public Position(double elevatorHeight, double armAngle) {
            this.elevatorHeight = elevatorHeight;
            this.armAngle = armAngle;
        }
    }
    
    private MoveState currentMoveState = MoveState.DONE;
    private Position targetPosition;
    
    public IntegratedMechanismSubsystem(ElevatorSubsystem elevator, ArmSubsystem arm) {
        this.elevator = elevator;
        this.arm = arm;
    }

    private boolean isArmAtZero() {
        return Math.abs(arm.getCurrentAngle()) < ANGLE_TOLERANCE;
    }

    private boolean isAboveSafeHeight() {
        return elevator.getCurrentHeight() >= ELEVATOR_SAFE_HEIGHT;
    }
    
    public void moveToPosition(Position position) {
        this.targetPosition = position;
        double currentHeight = elevator.getCurrentHeight();
        
        // Case 1: Moving to a position below safe height
        if (position.elevatorHeight < ELEVATOR_SAFE_HEIGHT) {
            if (!isAboveSafeHeight()) {
                // Need to move up to safe height first
                currentMoveState = MoveState.MOVING_TO_SAFE_HEIGHT;
                elevator.setHeight(ELEVATOR_SAFE_HEIGHT);
            } else if (!isArmAtZero()) {
                // Need to get arm to zero before we can go down
                currentMoveState = MoveState.WAITING_FOR_ARM_ZERO;
                arm.setAngle(ARM_SAFE_ANGLE);
            } else {
                // Arm is at zero and we're above safe height, can move down
                currentMoveState = MoveState.MOVING_ELEVATOR;
                elevator.setHeight(position.elevatorHeight);
            }
        }
        // Case 2: Current position below safe height, need to move up first
        else if (currentHeight < ELEVATOR_SAFE_HEIGHT) {
            currentMoveState = MoveState.MOVING_TO_SAFE_HEIGHT;
            elevator.setHeight(ELEVATOR_SAFE_HEIGHT);
        }
        // Case 3: Above safe height and staying above safe height
        else if (position.elevatorHeight >= ELEVATOR_SAFE_HEIGHT) {
            currentMoveState = MoveState.MOVING_TOGETHER;
            elevator.setHeight(position.elevatorHeight);
            arm.setAngle(position.armAngle);
        }
    }
    
    private void updateMovement() {
        SmartDashboard.putNumber("Debug/Current Height", elevator.getCurrentHeight());
        SmartDashboard.putNumber("Debug/Current Angle", arm.getCurrentAngle());
        SmartDashboard.putString("Debug/Current State", currentMoveState.toString());
        
        switch (currentMoveState) {
            case MOVING_TO_SAFE_HEIGHT:
                if (elevator.isAtTarget()) {
                    if (targetPosition.elevatorHeight < ELEVATOR_SAFE_HEIGHT) {
                        // If target is below safe height, need to get arm to zero
                        currentMoveState = MoveState.WAITING_FOR_ARM_ZERO;
                        arm.setAngle(ARM_SAFE_ANGLE);
                    } else {
                        // If target is above safe height, can move both
                        currentMoveState = MoveState.MOVING_TOGETHER;
                        elevator.setHeight(targetPosition.elevatorHeight);
                        arm.setAngle(targetPosition.armAngle);
                    }
                }
                break;
                
            case WAITING_FOR_ARM_ZERO:
                if (isArmAtZero()) {
                    // Arm is at zero, safe to move elevator down
                    currentMoveState = MoveState.MOVING_ELEVATOR;
                    elevator.setHeight(targetPosition.elevatorHeight);
                }
                break;
                
            case MOVING_ARM:
                if (arm.isAtTarget()) {
                    currentMoveState = MoveState.DONE;
                }
                break;
                
            case MOVING_ELEVATOR:
                if (elevator.isAtTarget()) {
                    if (targetPosition.armAngle != ARM_SAFE_ANGLE && isAboveSafeHeight()) {
                        currentMoveState = MoveState.MOVING_ARM;
                        arm.setAngle(targetPosition.armAngle);
                    } else {
                        currentMoveState = MoveState.DONE;
                    }
                }
                break;
                
            case MOVING_TOGETHER:
                if (elevator.isAtTarget() && arm.isAtTarget()) {
                    currentMoveState = MoveState.DONE;
                }
                break;
                
            case DONE:
                break;
        }
    }
    
    public boolean isAtTarget() {
        boolean atTarget = (currentMoveState == MoveState.DONE);
        SmartDashboard.putBoolean("Debug/Mechanism At Target", atTarget);
        return atTarget;
    }
    
    public void stop() {
        elevator.stop();
        arm.stop();
    }
    
    @Override
    public void periodic() {
        updateMovement();
        
        SmartDashboard.putNumber("Mechanism/Elevator Height", elevator.getCurrentHeight());
        SmartDashboard.putNumber("Mechanism/Arm Angle", arm.getCurrentAngle());
        SmartDashboard.putBoolean("Mechanism/At Target", isAtTarget());
        SmartDashboard.putString("Mechanism/Move State", currentMoveState.toString());
    }
}