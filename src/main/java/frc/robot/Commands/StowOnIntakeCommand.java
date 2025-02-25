package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SafetySubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;

public class StowOnIntakeCommand extends Command {
    private final SafetySubsystem safetySystem;
    private final AlgaeIntake algaeIntake;
    private final CoralIntake coralIntake;
    private boolean wasLastStateEmpty = true;
    private boolean hasInitialized = false;

    public StowOnIntakeCommand(SafetySubsystem safety, AlgaeIntake algae, CoralIntake coral) {
        this.safetySystem = safety;
        this.algaeIntake = algae;
        this.coralIntake = coral;
        addRequirements(safety);
    }

    @Override
    public void initialize() {
        // Always start in stowed position
        safetySystem.setTargetPosition(
            Constants.SafetyConstants.STOWED[0], 
            Constants.SafetyConstants.STOWED[1]
        );
        hasInitialized = true;
    }

    @Override
    public void execute() {
        boolean hasObject = algaeIntake.hasBall() || coralIntake.hasCoral();
        
        // Move to stow position if:
        // 1. We just got an object (state change from empty to full)
        // 2. We have an object and haven't finished initial stow
        if ((hasObject && wasLastStateEmpty) || 
            (hasObject && !safetySystem.isAtTarget())) {
            safetySystem.setTargetPosition(
                Constants.SafetyConstants.STOWED[0], 
                Constants.SafetyConstants.STOWED[1]
            );
        }
        
        wasLastStateEmpty = !hasObject;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}