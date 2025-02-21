package frc.robot.Commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.SafetySubsystem;
import frc.robot.Constants;

public class CoralIntakeCommand extends Command {
    private final CoralIntake coralIntake;
    private final SafetySubsystem safetySubsystem;
    private boolean hasStartedIntake = false;

    public CoralIntakeCommand(CoralIntake coralIntake, SafetySubsystem safetySubsystem) {
        this.coralIntake = coralIntake;
        this.safetySubsystem = safetySubsystem;
        addRequirements(coralIntake);
    }

    @Override
    public void initialize() {
        // Move to pickup position
        safetySubsystem.setTargetPosition(
            Constants.SafetyConstants.PICKUP[0], 
            Constants.SafetyConstants.PICKUP[1]
        );
        hasStartedIntake = false;
    }

    @Override
    public void execute() {
        // Start intaking if safety mechanism is at target and not already intaking
        if (safetySubsystem.isAtTarget() && !hasStartedIntake) {
            coralIntake.intakeCoral();
            hasStartedIntake = true;
        }

        // Check if coral is detected and stow if true
        if (coralIntake.hasCoral()) {
            safetySubsystem.setTargetPosition(
                Constants.SafetyConstants.STOWED[0], 
                Constants.SafetyConstants.STOWED[1]
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the intake and stow if not already stowed
        coralIntake.stop();
        if (!safetySubsystem.isAtTarget()) {
            safetySubsystem.setTargetPosition(
                Constants.SafetyConstants.STOWED[0], 
                Constants.SafetyConstants.STOWED[1]
            );
        }
    }

    @Override
    public boolean isFinished() {
        // Command ends when coral is detected and mechanism is stowed
        return coralIntake.hasCoral() && 
               safetySubsystem.isAtTarget();
    }
}