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

    public StowOnIntakeCommand(SafetySubsystem safety, AlgaeIntake algae, CoralIntake coral) {
        this.safetySystem = safety;
        this.algaeIntake = algae;
        this.coralIntake = coral;
        addRequirements(safety);
    }

    @Override
    public void execute() {
        boolean hasObject = algaeIntake.hasBall() || coralIntake.hasCoral();
        
        if (hasObject && wasLastStateEmpty) {
            safetySystem.setTargetPosition(Constants.SafetyConstants.STOWED[0], Constants.SafetyConstants.STOWED[1]); // Stow position
        }
        
        wasLastStateEmpty = !hasObject;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}