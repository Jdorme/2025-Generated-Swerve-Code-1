package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SafetySubsystem extends SubsystemBase {
    private static final double DANGER_ZONE_HEIGHT = 11; // inches
    private static final double SAFE_HEIGHT = DANGER_ZONE_HEIGHT + 2.0; // inches
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final CoralIntake coralIntake;
    private final AlgaeIntake algaeIntake;

    private double targetElevatorHeight = 0.0;
    private double targetArmAngle = 0.0;
    private boolean isMovingDownThroughDangerZone = false;

    public SafetySubsystem(ElevatorSubsystem elevator, ArmSubsystem arm, 
    CoralIntake coral, AlgaeIntake algae) {
this.elevator = elevator;
this.arm = arm;
this.coralIntake = coral;
this.algaeIntake = algae;
}

    public void setTargetPosition(double heightInches, double armAngleDegrees) {
        double currentHeight = elevator.getCurrentHeight();
        targetElevatorHeight = heightInches;
        targetArmAngle = armAngleDegrees;

        // Check if we're starting a downward movement through the danger zone
        if (currentHeight > DANGER_ZONE_HEIGHT && targetElevatorHeight <= DANGER_ZONE_HEIGHT) {
            // Only allow downward movement if arm is in safe position
            // Move arm to safe position if needed
            arm.setAngle(targetArmAngle);
            
            // Only proceed with elevator movement if arm is safe
            if (isArmSafe()) {
                isMovingDownThroughDangerZone = true;
                moveToTargets();
            }
        }
        // If we're already in the danger zone
        else if (isInDangerZone(currentHeight)) {
            if (targetElevatorHeight >= SAFE_HEIGHT) {
                // When moving up out of danger zone:
                // 1. First move elevator to safe height
                // 2. Only then allow arm movement
                isMovingDownThroughDangerZone = false;
                elevator.setHeight(SAFE_HEIGHT);
                
                if (elevator.getCurrentHeight() >= SAFE_HEIGHT) {
                    moveToTargets();
                }
            } else if (isMovingDownThroughDangerZone && isArmSafe()) {
                // Continue downward movement if it was intentional and arm is safe
                moveToTargets();
            } else {
                // Otherwise, maintain safe height
                elevator.setHeight(SAFE_HEIGHT);
            }
        }
        // Outside danger zone, normal operation
        else {
            isMovingDownThroughDangerZone = false;
            moveToTargets();
        }
    }

    private boolean isArmSafe() {
        // Since arm rotates parallel to elevator and only interfaces at pivot,
        // we just need to ensure it's stable at its commanded position
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

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Safety/In Danger Zone", isInDangerZone(elevator.getCurrentHeight()));
        SmartDashboard.putBoolean("Safety/At Target", isAtTarget());
        SmartDashboard.putBoolean("Safety/Moving Down Through Danger", isMovingDownThroughDangerZone);
    }
}