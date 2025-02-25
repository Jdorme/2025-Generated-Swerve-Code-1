package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SafetyConstants;
import frc.robot.subsystems.SafetySubsystem;
import frc.robot.subsystems.AlgaeIntake;

public class L2AlgaeCommand extends Command {
    private final SafetySubsystem m_safetySystem;
    private final AlgaeIntake m_algaeIntake;
    private boolean wasLastStateBallPresent = false;
    
    public L2AlgaeCommand(SafetySubsystem safetySystem, AlgaeIntake algaeIntake) {
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
            SafetyConstants.L2_ALGAE[0], 
            SafetyConstants.L2_ALGAE[1]
        );
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
            System.out.println("L2Algae: Ball acquired, holding position");
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
        
        // Return to stowed position
        System.out.println("L2Algae: Returning to stowed position");

    }

    @Override
    public boolean isFinished() {
        return false;  // Runs until button is released
    }
}