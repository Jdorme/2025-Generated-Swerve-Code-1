package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SafetyConstants;
import frc.robot.subsystems.SafetySubsystem;
import frc.robot.subsystems.AlgaeIntake;

public class ProcessorCommand extends Command {
    private final SafetySubsystem m_safetySystem;
    private final AlgaeIntake m_algaeIntake;
    private boolean wasLastStateBallPresent = false;
    private boolean isAtL2Position = false;
    private boolean movedToProcessor = false;
    private long l2ReachedTimestamp = 0;
    private static final long DWELL_TIME_MS = 500; // Time to wait at L2 position before going to processor
    
    public ProcessorCommand(SafetySubsystem safetySystem, AlgaeIntake algaeIntake) {
        m_safetySystem = safetySystem;
        m_algaeIntake = algaeIntake;
        addRequirements(safetySystem, algaeIntake);
    }

    @Override
    public void initialize() {
        // Reset state tracking
        wasLastStateBallPresent = false;
        isAtL2Position = false;
        movedToProcessor = false;
        l2ReachedTimestamp = 0;
        
        // First, move to L2 position
        m_safetySystem.setTargetPosition(
            SafetyConstants.L2_ALGAE[0], 
            SafetyConstants.L2_ALGAE[1]
        );
        
        System.out.println("Processor: First moving to L2 position");
    }

    @Override
    public void execute() {
        boolean hasBall = m_algaeIntake.hasBall();
        
        // First stage: Get to L2 position
        if (!isAtL2Position) {
            // Check if we've reached L2 position
            if (m_safetySystem.isAtTarget()) {
                if (l2ReachedTimestamp == 0) {
                    // Just reached L2 position, record the time
                    l2ReachedTimestamp = System.currentTimeMillis();
                    System.out.println("Processor: Reached L2 position, dwelling for stability");
                } else if (System.currentTimeMillis() - l2ReachedTimestamp >= DWELL_TIME_MS) {
                    // We've been at L2 position for the required dwell time
                    isAtL2Position = true;
                    System.out.println("Processor: L2 position stable, now moving to processor position");
                    
                    // Now move to processor position
                    m_safetySystem.setTargetPosition(
                        SafetyConstants.PROCESSOR_ALGAE[0], 
                        SafetyConstants.PROCESSOR_ALGAE[1]
                    );
                    movedToProcessor = true;
                }
            }
        }
        // Second stage: At processor position, handle intake
        else if (movedToProcessor) {
            // If we're in position and don't have a ball yet, run intake
            if (m_safetySystem.isAtTarget() && !hasBall) {
                m_algaeIntake.intake();
            }
            
            // If we just got a ball, output to console for debugging
            if (hasBall && !wasLastStateBallPresent) {
                System.out.println("Processor: Ball acquired, maintaining position");
            }
        }
        
        // Update last state
        wasLastStateBallPresent = hasBall;

        // Log state to SmartDashboard
        SmartDashboard.putBoolean("Processor/AtPosition", m_safetySystem.isAtTarget() && isAtL2Position && movedToProcessor);
        SmartDashboard.putBoolean("Processor/HasBall", hasBall);
        SmartDashboard.putBoolean("Processor/AtL2", isAtL2Position);
        SmartDashboard.putBoolean("Processor/MovedToProcessor", movedToProcessor);
        SmartDashboard.putString("Processor/State", 
            !isAtL2Position ? "MOVING_TO_L2" : 
            (movedToProcessor ? (hasBall ? "HOLDING" : "INTAKING") : "MOVING_TO_PROCESSOR"));
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Processor: Command interrupted");
        }
        
        // If we don't have a ball, stop the intake
        if (!m_algaeIntake.hasBall()) {
            System.out.println("Processor: No ball held, stopping intake");
            m_algaeIntake.stop();
        } else {
            System.out.println("Processor: Ball held, maintaining hold mode");
            // The AlgaeIntake will automatically maintain hold mode
        }
        
        // No longer returning to stowed position automatically
        System.out.println("Processor: Maintaining current position until next command");
    }

    @Override
    public boolean isFinished() {
        // Never finish on its own - will run until interrupted by another command
        return false;
    }
}