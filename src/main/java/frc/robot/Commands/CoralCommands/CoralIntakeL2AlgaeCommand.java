package frc.robot.Commands.CoralCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.SafetySubsystem;
import frc.robot.Constants;

public class CoralIntakeL2AlgaeCommand extends Command {
    private final CoralIntake coralIntake;
    private final SafetySubsystem safetySubsystem;
    private boolean hasStartedIntake = false;
    private boolean hasDetectedCoral = false;
    private boolean hasStartedHoldBack = false;
    private Timer coralDetectionTimer = new Timer();
    private Timer holdBackTimer = new Timer();
    private final double INTAKE_CONTINUE_DURATION = 0.5; // Half a second
    private final double HOLD_BACK_DURATION = 1; // Time to hold coral at the back

    public CoralIntakeL2AlgaeCommand(CoralIntake coralIntake, SafetySubsystem safetySubsystem) {
        this.coralIntake = coralIntake;
        this.safetySubsystem = safetySubsystem;
        addRequirements(coralIntake, safetySubsystem);
    }

    @Override
    public void initialize() {
        // Move to pickup position
        safetySubsystem.setTargetPosition(
            Constants.SafetyConstants.PICKUP[0], 
            Constants.SafetyConstants.PICKUP[1]
            
        );
        coralIntake.intakeCoral();
        hasStartedIntake = false;
        hasDetectedCoral = false;
        hasStartedHoldBack = false;
        coralDetectionTimer.reset();
        holdBackTimer.reset();
    }

    @Override
    public void execute() {
        // Start intaking if safety mechanism is at target and not already intaking
        if (safetySubsystem.isAtTarget() && !hasStartedIntake) {
            coralIntake.intakeCoral();
            hasStartedIntake = true;
        }

        // Check if coral is detected and start timer if true
        if (coralIntake.hasCoral() && !hasDetectedCoral) {
            hasDetectedCoral = true;
            coralDetectionTimer.start();
            
            // Start moving to stow position immediately
            safetySubsystem.setTargetPosition(
                Constants.SafetyConstants.STOWED[0], 
                Constants.SafetyConstants.STOWED[1]
            );
        }
        
        // Hold the coral back at the end of the intake
        if (hasDetectedCoral && coralDetectionTimer.hasElapsed(INTAKE_CONTINUE_DURATION) && !hasStartedHoldBack) {
            coralIntake.holdBack(); // Use the new holdBack feature
            hasStartedHoldBack = true;
            holdBackTimer.start();
        }
        
        // After holding back for a short duration, go to normal hold
        if (hasStartedHoldBack && holdBackTimer.hasElapsed(HOLD_BACK_DURATION)) {
            coralIntake.normalHold();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Return to normal hold if we have coral, otherwise stop the intake
        if (coralIntake.hasCoral()) {
            coralIntake.normalHold();
        } else {
            coralIntake.stop();
        }
        
        // Stow if not already stowed
        if (!safetySubsystem.isAtTarget()) {
            safetySubsystem.setTargetPosition(
                Constants.SafetyConstants.STOWED[0], 
                Constants.SafetyConstants.STOWED[1]
            );
        }
    }

    @Override
    public boolean isFinished() {
        // Command ends when coral is detected, both timers have elapsed, and mechanism is stowed
        return hasDetectedCoral && 
               coralDetectionTimer.hasElapsed(INTAKE_CONTINUE_DURATION) &&
               hasStartedHoldBack &&
               holdBackTimer.hasElapsed(HOLD_BACK_DURATION) &&
               safetySubsystem.isAtTarget();
    }
}