package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SafetyConstants;

public class SafetySubsystem extends SubsystemBase {
    // Safety thresholds moved to named constants for clarity
    private static final double DANGER_ZONE_HEIGHT = 11.0; // inches
    private static final double SAFETY_MARGIN = 2.0; // inches
    private static final double SAFE_HEIGHT = DANGER_ZONE_HEIGHT + SAFETY_MARGIN;
    
    // Subsystem dependencies
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final CoralIntake coralIntake;
    private final AlgaeIntake algaeIntake;

    // State tracking
    private Position currentTargetPosition = new Position(0, 0);
    private boolean isMovingDownThroughDangerZone = false;

    // Position record class for cleaner code
    public static class Position {
        public final double elevatorHeight;
        public final double armAngle;

        public Position(double elevatorHeight, double armAngle) {
            this.elevatorHeight = elevatorHeight;
            this.armAngle = armAngle;
        }

        @Override
        public String toString() {
            return String.format("(Height: %.2f, Angle: %.2f)", elevatorHeight, armAngle);
        }
    }

    public SafetySubsystem(ElevatorSubsystem elevator, ArmSubsystem arm, 
                          CoralIntake coral, AlgaeIntake algae) {
        this.elevator = elevator;
        this.arm = arm;
        this.coralIntake = coral;
        this.algaeIntake = algae;
    }

    public void setTargetPosition(double heightInches, double armAngleDegrees) {
        currentTargetPosition = new Position(heightInches, armAngleDegrees);
        updateMovementWithSafety();
    }

    private void updateMovementWithSafety() {
        double currentHeight = elevator.getCurrentHeight();
        boolean isCurrentlyInDangerZone = isInDangerZone(currentHeight);
        boolean isTargetInDangerZone = isInDangerZone(currentTargetPosition.elevatorHeight);

        if (isMovingDownIntoOrThroughDangerZone(currentHeight)) {
            handleDownwardMovementThroughDangerZone();
        } else if (isCurrentlyInDangerZone) {
            handleMovementFromDangerZone(isTargetInDangerZone);
        } else {
            // Outside danger zone, normal operation
            moveToTargets();
        }
    }

    private boolean isMovingDownIntoOrThroughDangerZone(double currentHeight) {
        return currentHeight > DANGER_ZONE_HEIGHT && 
               currentTargetPosition.elevatorHeight <= DANGER_ZONE_HEIGHT;
    }

    private void handleDownwardMovementThroughDangerZone() {
        // Move arm to target position first
        arm.setAngle(currentTargetPosition.armAngle);
        
        // Only proceed with elevator movement if arm is in position
        if (arm.isAtTarget()) {
            isMovingDownThroughDangerZone = true;
            moveToTargets();
        }
    }

    private void handleMovementFromDangerZone(boolean isTargetInDangerZone) {
        if (currentTargetPosition.elevatorHeight >= SAFE_HEIGHT) {
            // Moving up out of danger zone - move elevator first
            elevator.setHeight(SAFE_HEIGHT);
            
            if (elevator.getCurrentHeight() >= SAFE_HEIGHT) {
                moveToTargets();
            }
        } else if (isMovingDownThroughDangerZone && isArmSafe()) {
            // Continuing planned downward movement
            moveToTargets();
        } else {
            // Maintain safe height if movement isn't explicitly allowed
            elevator.setHeight(SAFE_HEIGHT);
        }
    }

    private boolean isArmSafe() {
        return arm.isAtTarget();
    }

    private void moveToTargets() {
        elevator.setHeight(currentTargetPosition.elevatorHeight);
        arm.setAngle(currentTargetPosition.armAngle);
    }

    private boolean isInDangerZone(double height) {
        return height <= DANGER_ZONE_HEIGHT;
    }

    public boolean isAtTarget() {
        return elevator.isAtTarget() && arm.isAtTarget();
    }

    @Override
    public void periodic() {
        // Log current state and safety information
        SmartDashboard.putBoolean("Safety/In Danger Zone", 
            isInDangerZone(elevator.getCurrentHeight()));
        SmartDashboard.putBoolean("Safety/At Target", isAtTarget());
        SmartDashboard.putBoolean("Safety/Moving Down Through Danger", 
            isMovingDownThroughDangerZone);
        SmartDashboard.putString("Safety/Target Position", 
            currentTargetPosition.toString());
    }
}