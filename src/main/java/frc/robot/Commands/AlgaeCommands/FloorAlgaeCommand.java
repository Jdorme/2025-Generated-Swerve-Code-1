package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SafetyConstants;
import frc.robot.subsystems.SafetySubsystem;
import frc.robot.subsystems.AlgaeIntake;

public class FloorAlgaeCommand extends Command {
    private final SafetySubsystem m_safetySystem;
    private final AlgaeIntake m_algaeIntake;
    private boolean wasLastStateBallPresent = false;
    
    public FloorAlgaeCommand(SafetySubsystem safetySystem, AlgaeIntake algaeIntake) {
        m_safetySystem = safetySystem;
        m_algaeIntake = algaeIntake;
        addRequirements(safetySystem, algaeIntake);
    }

    @Override
    public void initialize() {
        // Reset state tracking
        wasLastStateBallPresent = false;
        
        // Move to ground position using safety system
        m_safetySystem.setTargetPosition(
            SafetyConstants.GROUND_ALGAE[0], 
            SafetyConstants.GROUND_ALGAE[1]
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
            System.out.println("FloorAlgae: Ball acquired, holding position");
        }
        
        // Update last state
        wasLastStateBallPresent = hasBall;

        // Log state to SmartDashboard
        SmartDashboard.putBoolean("FloorAlgae/AtPosition", m_safetySystem.isAtTarget());
        SmartDashboard.putBoolean("FloorAlgae/HasBall", hasBall);
        SmartDashboard.putString("FloorAlgae/State", hasBall ? "HOLDING" : "INTAKING");
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("FloorAlgae: Command interrupted");
        }
        
        // If we don't have a ball, stop the intake
        if (!m_algaeIntake.hasBall()) {
            System.out.println("FloorAlgae: No ball held, stopping intake");
            m_algaeIntake.stop();
        } else {
            System.out.println("FloorAlgae: Ball held, maintaining hold mode");
            // The AlgaeIntake will automatically maintain hold mode
        }
        
        // Return to stowed position
        System.out.println("FloorAlgae: Returning to stowed position");
        m_safetySystem.setTargetPosition(
            SafetyConstants.STOWED[0], 
            SafetyConstants.STOWED[1]
        );
    }

    @Override
    public boolean isFinished() {
        return false;  // Runs until button is released
    }
}