package frc.robot.Commands.CoralCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.SafetySubsystem;
import frc.robot.Constants;

public class AutoCoralIntakeCommand extends Command {
    private final CoralIntake coralIntake;
    private final SafetySubsystem safetySubsystem;
    private boolean hasStartedIntake = false;
    private boolean hasDetectedCoral = false;
    private boolean hasStartedHoldBack = false;
    private boolean hasSetTargetPosition = false;
    private Timer coralDetectionTimer = new Timer();
    private Timer holdBackTimer = new Timer();
    private Timer initialDelayTimer = new Timer();
    private final double INTAKE_CONTINUE_DURATION = 0.5; // Half a second
    private final double HOLD_BACK_DURATION = 0; // Time to hold coral at the back
    private final double INITIAL_DELAY_DURATION = 1; // .5 second initial delay
    private boolean hasDelayElapsed = false;

    public AutoCoralIntakeCommand(CoralIntake coralIntake, SafetySubsystem safetySubsystem) {
        this.coralIntake = coralIntake;
        this.safetySubsystem = safetySubsystem;
        addRequirements(coralIntake);
    }

    @Override
    public void initialize() {
        // Do NOT move to pickup position immediately
        // We'll wait for the delay first
        hasStartedIntake = false;
        hasDetectedCoral = false;
        hasStartedHoldBack = false;
        hasDelayElapsed = false;
        hasSetTargetPosition = false;
        coralDetectionTimer.reset();
        holdBackTimer.reset();
        initialDelayTimer.reset();
        initialDelayTimer.start();
    }

    @Override
    public void execute() {
            // Wait for the initial delay to elapse
    if (!hasDelayElapsed && initialDelayTimer.hasElapsed(INITIAL_DELAY_DURATION)) {
        hasDelayElapsed = true;
        // Only set target position after delay has elapsed
        safetySubsystem.setTargetPosition(
            Constants.SafetyConstants.PICKUP[0], 
            Constants.SafetyConstants.PICKUP[1]
        );
        hasSetTargetPosition = true;
    }
    
    // Only proceed with normal execution after the delay has elapsed
    if (!hasDelayElapsed) {
        return;
    }

    // Start intaking if safety mechanism is at target and not already intaking
    if (hasSetTargetPosition && safetySubsystem.isAtTarget() && !hasStartedIntake) {
        coralIntake.intakeCoral();
        hasStartedIntake = true;
    }

    // Check if coral is detected
    if (coralIntake.hasCoral() && !hasDetectedCoral) {
        hasDetectedCoral = true;
        
        // Start moving to stow position immediately
        safetySubsystem.setTargetPosition(
            Constants.SafetyConstants.STOWED[0], 
            Constants.SafetyConstants.STOWED[1]
        );
        
        // Use normalHold directly instead of using holdBack
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
        return hasDelayElapsed && hasDetectedCoral;
    }
}