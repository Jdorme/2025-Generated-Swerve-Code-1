package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {
    private final ArmSubsystem m_arm;
    private final double m_targetAngle;

    public ArmCommand(ArmSubsystem arm, double targetAngle) {
        m_arm = arm;
        m_targetAngle = targetAngle;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.setAngle(m_targetAngle);
    }

    @Override
    public boolean isFinished() {
        return m_arm.isAtTarget();
    }
}