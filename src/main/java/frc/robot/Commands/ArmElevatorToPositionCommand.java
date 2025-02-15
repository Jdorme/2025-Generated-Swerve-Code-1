package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SafetySubsystem;

public class ArmElevatorToPositionCommand extends Command {
    private final SafetySubsystem safety;
    private final double height;
    private final double angle;

    public ArmElevatorToPositionCommand(SafetySubsystem safety, double height, double angle) {
        this.safety = safety;
        this.height = height;
        this.angle = angle;
        addRequirements(safety);  // This is key - it will interrupt the default command
    }

    @Override
    public void initialize() {
        safety.setTargetPosition(height, angle);
    }

    @Override
    public boolean isFinished() {
        return safety.isAtTarget();
    }
}