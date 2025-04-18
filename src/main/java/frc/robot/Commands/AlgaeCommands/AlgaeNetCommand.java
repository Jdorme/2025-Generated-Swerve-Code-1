package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SafetyConstants;
import frc.robot.subsystems.SafetySubsystem;
import frc.robot.subsystems.AlgaeIntake;

public class AlgaeNetCommand extends Command {
    private final SafetySubsystem m_safetySystem;
    private final AlgaeIntake m_algaeIntake;
    private boolean wasLastStateBallPresent = false;
    
    public AlgaeNetCommand(SafetySubsystem safetySystem, AlgaeIntake algaeIntake) {
        m_safetySystem = safetySystem;
        m_algaeIntake = algaeIntake;
        addRequirements(safetySystem, algaeIntake);
    }

    @Override
    public void initialize() {
        // Reset state tracking
        wasLastStateBallPresent = false;
        
        // Move to L2 algae position using safety system
        m_safetySystem.setTargetPosition(
            SafetyConstants.NET_ALGAE[0], 
            SafetyConstants.NET_ALGAE[1]
        );
        
        System.out.println("L2Algae: Moving to position to intake algae");
    }

    @Override
    public void execute() {
        boolean hasBall = m_algaeIntake.hasBall();
        
        // If we're in position and don't have a ball yet, run intake
        if (m_safetySystem.isAtTarget() && !hasBall) {
            m_algaeIntake.intake();
        }
        
        // If we just got a ball, output to console for debugging
        if (hasBall && !wasLastStateBallPresent) {
            System.out.println("L2Algae: Ball acquired, maintaining position");
        }
        
        // Update last state
        wasLastStateBallPresent = hasBall;

        // Log state to SmartDashboard
        SmartDashboard.putBoolean("L2Algae/AtPosition", m_safetySystem.isAtTarget());
        SmartDashboard.putBoolean("L2Algae/HasBall", hasBall);
        SmartDashboard.putString("L2Algae/State", hasBall ? "HOLDING" : "INTAKING");
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("L2Algae: Command interrupted");
        }
        
        // If we don't have a ball, stop the intake
        if (!m_algaeIntake.hasBall()) {
            System.out.println("L2Algae: No ball held, stopping intake");
            m_algaeIntake.stop();
        } else {
            System.out.println("L2Algae: Ball held, maintaining hold mode");
            // The AlgaeIntake will automatically maintain hold mode
        }
        
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