package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SafetySubsystem extends SubsystemBase {
    private static final double DANGER_ZONE_HEIGHT = 4; // inches
    private static final double STANDARD_SAFE_HEIGHT = DANGER_ZONE_HEIGHT + 0; // inches
    private static final double ALGAE_SAFE_HEIGHT = DANGER_ZONE_HEIGHT + 0; // Higher safe height for algae
    
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final CoralIntake coralIntake;
    private final AlgaeIntake algaeIntake;

    private double targetElevatorHeight = 0.0;
    private double targetArmAngle = 0.0;
    private boolean isMovingDownThroughDangerZone = false;
    private boolean isMovingUpFromDangerZone = false;
    private boolean isInTransition = false;
    private boolean overrideEnabled = false;

    public SafetySubsystem(ElevatorSubsystem elevator, ArmSubsystem arm, 
                          CoralIntake coral, AlgaeIntake algae) {
        this.elevator = elevator;
        this.arm = arm;
        this.coralIntake = coral;
        this.algaeIntake = algae;
    }

    private double getCurrentSafeHeight() {
        // Use higher safe height if holding algae
        return algaeIntake.hasBall() ? ALGAE_SAFE_HEIGHT : STANDARD_SAFE_HEIGHT;
    }

    public void setTargetPosition(double heightInches, double armAngleDegrees) {
        double currentHeight = elevator.getCurrentHeight();
        targetElevatorHeight = heightInches;
        targetArmAngle = armAngleDegrees;
        boolean currentlyInDangerZone = isInDangerZone(currentHeight);
        double safeHeight = getCurrentSafeHeight();

        // Case 1: Starting ABOVE danger zone, moving DOWN through it
        if (!currentlyInDangerZone && targetElevatorHeight <= DANGER_ZONE_HEIGHT) {
            // First ensure arm is in safe position
            arm.setAngle(targetArmAngle);
            
            // Only proceed with downward movement if arm is at target
            if (arm.isAtTarget()) {
                isMovingDownThroughDangerZone = true;
                isMovingUpFromDangerZone = false;
                isInTransition = false;
                moveToTargets();
            } else {
                // If arm isn't at target yet, don't move elevator down
                isInTransition = true;
            }
        } 
        // Case 2: Currently IN danger zone
        else if (currentlyInDangerZone) {
            // Case 2A: Want to move UP out of danger zone
            if (targetElevatorHeight >= safeHeight) {
                isMovingUpFromDangerZone = true;
                isMovingDownThroughDangerZone = false;
                isInTransition = false;
                
                // First move elevator up to safe height
                elevator.setHeight(safeHeight);
                
                // Only move arm once elevator is at safe height
                if (elevator.getCurrentHeight() >= safeHeight - 0.5) { // Allow some tolerance
                    arm.setAngle(targetArmAngle);
                    
                    // Once arm is at target, complete the movement to final position
                    if (arm.isAtTarget() && Math.abs(elevator.getCurrentHeight() - safeHeight) < 0.5) {
                        elevator.setHeight(targetElevatorHeight);
                    }
                }
            } 
            // Case 2B: Want to move to another position WITHIN danger zone
            else if (targetElevatorHeight <= DANGER_ZONE_HEIGHT) {
                // Only allow this if arm is in safe position or override is enabled
                if (arm.isAtTarget() || overrideEnabled) {
                    moveToTargets();
                } else {
                    // Move arm to safe position first
                    arm.setAngle(targetArmAngle);
                    isInTransition = true;
                }
            }
        } 
        // Case 3: Normal operation (outside danger zone)
        else {
            // Check if target requires passing through danger zone or significant arm movement
            boolean targetIsInDangerZone = isInDangerZone(targetElevatorHeight);
            
            // If target is in danger zone or requires arm movement first
            if (targetIsInDangerZone || needsArmMovementFirst(currentHeight, targetElevatorHeight, targetArmAngle)) {
                // Move arm first for safety
                arm.setAngle(targetArmAngle);
                
                if (arm.isAtTarget()) {
                    isInTransition = false;
                    elevator.setHeight(targetElevatorHeight);
                } else {
                    isInTransition = true;
                }
            } else {
                // For truly safe transitions, still sequence the movements 
                // (arm first, then elevator) to maintain consistent behavior
                arm.setAngle(targetArmAngle);
                
                if (arm.isAtTarget()) {
                    isMovingDownThroughDangerZone = false;
                    isMovingUpFromDangerZone = false;
                    isInTransition = false;
                    elevator.setHeight(targetElevatorHeight);
                } else {
                    isInTransition = true;
                }
            }
        }
    }
    
    // Helper method to determine if arm should move first
    private boolean needsArmMovementFirst(double currentHeight, double targetHeight, double targetAngle) {
        // Logic to determine if the arm should move first
        // This could be based on arm extension, target position, etc.
        
        // Check if arm needs significant movement (more than 5 degrees)
        boolean significantArmMovement = Math.abs(arm.getCurrentAngle() - targetAngle) > 5.0;
        
        // Always prioritize arm movement when starting from stow position
        // or when significant arm angle change is needed
        return significantArmMovement;
    }

    private boolean isArmSafe() {
        return arm.isAtTarget();
    }

    private void moveToTargets() {
        elevator.setHeight(targetElevatorHeight);
        arm.setAngle(targetArmAngle);
    }

    private boolean isInDangerZone(double height) {
        return height <= DANGER_ZONE_HEIGHT;
    }

    public boolean isAtTarget() {
        return elevator.isAtTarget() && arm.isAtTarget();
    }
    
    // Enable override for emergency situations - use with caution
    public void enableOverride(boolean enable) {
        overrideEnabled = enable;
        SmartDashboard.putBoolean("Safety/Override", overrideEnabled);
    }

    @Override
    public void periodic() {
        double currentHeight = elevator.getCurrentHeight();
        boolean currentlyInDangerZone = isInDangerZone(currentHeight);
        
        // If we're moving up from danger zone and are now safely above it
        if (isMovingUpFromDangerZone && !currentlyInDangerZone) {
            // Complete the movement to final position if not already there
            if (Math.abs(currentHeight - targetElevatorHeight) > 0.5) {
                elevator.setHeight(targetElevatorHeight);
            }
            
            // If we've reached the target position, clear the transition flag
            if (Math.abs(currentHeight - targetElevatorHeight) < 0.5 && arm.isAtTarget()) {
                isMovingUpFromDangerZone = false;
            }
        }
        
        // If we were in transition waiting for arm to reach position
        if (isInTransition && arm.isAtTarget()) {
            isInTransition = false;
            // Now that arm is in position, we can move elevator
            elevator.setHeight(targetElevatorHeight);
        }
        
        // Update dashboard data
        SmartDashboard.putBoolean("Safety/In Danger Zone", currentlyInDangerZone);
        SmartDashboard.putBoolean("Safety/At Target", isAtTarget());
        SmartDashboard.putBoolean("Safety/Moving Down Through Danger", isMovingDownThroughDangerZone);
        SmartDashboard.putBoolean("Safety/Moving Up From Danger", isMovingUpFromDangerZone);
        SmartDashboard.putBoolean("Safety/In Transition", isInTransition);
        SmartDashboard.putNumber("Safety/Current Safe Height", getCurrentSafeHeight());
        SmartDashboard.putNumber("Safety/Target Elevator Height", targetElevatorHeight);
        SmartDashboard.putNumber("Safety/Target Arm Angle", targetArmAngle);
        SmartDashboard.putBoolean("Safety/Has Algae", algaeIntake.hasBall());
    }
}