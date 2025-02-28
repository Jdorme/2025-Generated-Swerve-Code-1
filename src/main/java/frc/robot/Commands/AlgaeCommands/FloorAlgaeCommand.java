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
        
        System.out.println("FloorAlgae: Moving to position to intake algae");
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
            System.out.println("FloorAlgae: Ball acquired, maintaining position");
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
        
        // No longer returning to stowed position automatically
        // This is the key change - we don't move back to stowed until another command requires it
        System.out.println("FloorAlgae: Maintaining current position until next command");
    }

    @Override
    public boolean isFinished() {
        // Never finish on its own - will run until interrupted by another command
        // This means it won't stop when the button is released
        return false;
    }
}