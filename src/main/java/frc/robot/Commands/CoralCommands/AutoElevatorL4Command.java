package frc.robot.Commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SafetyConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoElevatorL4Command extends Command {
    private final ElevatorSubsystem m_elevator;
    
    // Position tolerance
    private static final double ELEVATOR_TOLERANCE = 0.5; // inches
    private boolean hasReachedTarget = false;
    
    // Delay variables
    private static final double DELAY_SECONDS = .5; // 1 second delay
    private double startTime;
    private boolean delayComplete = false;
    
    public AutoElevatorL4Command(ElevatorSubsystem elevator) {
        m_elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorL4: Starting command with 1 second delay");
        hasReachedTarget = false;
        delayComplete = false;
        startTime = System.currentTimeMillis() / 1000.0; // Current time in seconds
    }

    @Override
    public void execute() {
        // Check if delay period is complete
        double currentTime = System.currentTimeMillis() / 1000.0;
        double elapsedTime = currentTime - startTime;
        
        SmartDashboard.putNumber("ElevatorL4/DelayTimeElapsed", elapsedTime);
        
        // If we're still in delay period, don't do anything else
        if (!delayComplete) {
            if (elapsedTime >= DELAY_SECONDS) {
                delayComplete = true;
                System.out.println("ElevatorL4: Delay complete, now setting elevator height");
                // Set elevator to L4 height after delay
                m_elevator.setHeight(SafetyConstants.L4[0]);
            }
            return;
        }
        
        // After delay, update dashboard with current information
        double currentHeight = m_elevator.getCurrentHeight();
        double targetHeight = SafetyConstants.L4[0];
        
        SmartDashboard.putNumber("ElevatorL4/CurrentHeight", currentHeight);
        SmartDashboard.putNumber("ElevatorL4/TargetHeight", targetHeight);
        SmartDashboard.putNumber("ElevatorL4/HeightError", Math.abs(currentHeight - targetHeight));
        
        // Check if we've reached target and set flag
        if (!hasReachedTarget && Math.abs(currentHeight - targetHeight) <= ELEVATOR_TOLERANCE) {
            hasReachedTarget = true;
            System.out.println("ElevatorL4: Reached target height, maintaining position");
        }
    }

    @Override
    public boolean isFinished() {
        // Command never finishes on its own - it will maintain the L4 height
        // until another command takes control of the elevator subsystem
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("ElevatorL4: Command interrupted, elevator will maintain current position");
        }
        // No "completed normally" case since this command doesn't finish on its own
    }
}