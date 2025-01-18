package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualElevatorTest extends Command {
    private final ElevatorSubsystem m_elevator;
    private final double m_power;

    public ManualElevatorTest(ElevatorSubsystem elevator, double power) {
        m_elevator = elevator;
        m_power = power;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        System.out.println("Starting manual elevator test with power: " + m_power);
    }

    @Override
    public void execute() {
        m_elevator.setMotorOutput(m_power);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.setMotorOutput(0);
        System.out.println("Ending manual elevator test");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}