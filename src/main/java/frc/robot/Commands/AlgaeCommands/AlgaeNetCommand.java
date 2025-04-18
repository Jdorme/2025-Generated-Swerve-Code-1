package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SafetyConstants;
import frc.robot.subsystems.SafetySubsystem;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AlgaeNetCommand extends Command {
    
  private final ElevatorSubsystem m_elevatorSubsystem;
    private final ArmSubsystem m_armSubsystem;
    private final AlgaeIntake m_algaeIntake;
    private boolean wasLastStateBallPresent = false;
    
    public AlgaeNetCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, AlgaeIntake algaeIntake) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_armSubsystem = armSubsystem;
        m_algaeIntake = algaeIntake;
        addRequirements(elevatorSubsystem, armSubsystem, algaeIntake);
    }

    @Override
    public void initialize() {
        // Reset state tracking
        wasLastStateBallPresent = false;
        
        // Move to L2 algae position using safety system
          m_elevatorSubsystem.setHeight(SafetyConstants.NET_ALGAE[0]);
          m_armSubsystem.setAngle(SafetyConstants.NET_ALGAE[1]);

        
        System.out.println("L2Algae: Moving to position to intake algae");
    }

    @Override
    public void execute() {
        boolean hasBall = m_algaeIntake.hasBall();
        m_elevatorSubsystem.setHeight(SafetyConstants.NET_ALGAE[0]);
          m_armSubsystem.setAngle(SafetyConstants.NET_ALGAE[1]);
        System.out.println("AlgaeNet executing - Elevator height: " + m_elevatorSubsystem.getCurrentHeight() + 
        " vs target: " + SafetyConstants.NET_ALGAE[0] + 
        ", Arm angle: " + m_armSubsystem.getCurrentAngle() + 
        " vs target: " + SafetyConstants.NET_ALGAE[1]);



        // If we're in position and don't have a ball yet, run intake
        if (m_elevatorSubsystem.isAtTarget() && m_armSubsystem.isAtTarget()) {
            m_algaeIntake.intake();
        }
        
        
        //
        // Log state to SmartDashboard
        SmartDashboard.putBoolean("L2Algae/AtPosition", m_elevatorSubsystem.isAtTarget() && m_armSubsystem.isAtTarget());
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("L2Algae: Command interrupted");
        }
        
        // If we don't have a ball, stop the intake
            m_algaeIntake.stop();

        
        // No longer returning to stowed position automatically
        // This is the key change - we don't move back to stowed until another command requires it
        System.out.println("L2Algae: Maintaining current position until next command");
    }

    @Override
    public boolean isFinished() {
        // Never finish on its own - will run until interrupted by another command
        // This means it won't stop when the button is released
        return false;
    }
}