package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntegratedMechanismSubsystem;
import frc.robot.subsystems.IntegratedMechanismSubsystem.Position;

public class IntegratedMechanismCommand extends Command {
    private final IntegratedMechanismSubsystem mechanism;
    private final Position targetPosition;
    
    public IntegratedMechanismCommand(IntegratedMechanismSubsystem mechanism, Position targetPosition) {
        this.mechanism = mechanism;
        this.targetPosition = targetPosition;
        addRequirements(mechanism);
    }
    
    @Override
    public void initialize() {
        mechanism.moveToPosition(targetPosition);
    }
    
    @Override
    public boolean isFinished() {
        return mechanism.isAtTarget();
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            mechanism.stop();
        }
    }
}