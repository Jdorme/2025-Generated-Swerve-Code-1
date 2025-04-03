package frc.robot.Commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SafetyConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralIntake;

public class AutoElevatorArmL4Command extends Command {
    private enum SetupState {
        WAITING_FOR_DELAY,     // Initial delay before starting
        ELEVATOR_UP_ARM_TO_SCORE,        
        READY_TO_SCORE,        // Everything in position, ready for coral ejection
    }
    
    private final ElevatorSubsystem m_elevator;
    private final ArmSubsystem m_arm;
    private final CoralIntake m_coralIntake;
    private SetupState currentState = SetupState.WAITING_FOR_DELAY;
    
    // Position tolerances
    private static final double ELEVATOR_TOLERANCE = 0.5; // inches
    private static final double ARM_TOLERANCE = 2.0; // degrees
    
    // Delay variables
    private static final double DELAY_SECONDS = .25; // 0.5 second delay
    private double startTime;
    
    public AutoElevatorArmL4Command(ElevatorSubsystem elevator, ArmSubsystem arm, CoralIntake coralIntake) {
        m_elevator = elevator;
        m_arm = arm;
        m_coralIntake = coralIntake;
        addRequirements(elevator, arm);
    }

    @Override
    public void initialize() {
        //System.out.println("ElevatorArmL4: Starting command with delay");
        currentState = SetupState.WAITING_FOR_DELAY;
        startTime = System.currentTimeMillis() / 1000.0; // Current time in seconds
        
        // Ensure arm is at zero initially while we wait
        m_arm.setAngle(0);
        m_coralIntake.setIntakeSpeed(.06);
        
    }

    private boolean isElevatorAtTarget() {
        double currentHeight = m_elevator.getCurrentHeight();
        double targetHeight = SafetyConstants.L4[0];
        boolean atTarget = Math.abs(currentHeight - targetHeight) <= ELEVATOR_TOLERANCE;
        
        SmartDashboard.putNumber("ElevatorArmL4/CurrentHeight", currentHeight);
        SmartDashboard.putNumber("ElevatorArmL4/TargetHeight", targetHeight);
        SmartDashboard.putNumber("ElevatorArmL4/HeightError", Math.abs(currentHeight - targetHeight));
        
        return atTarget;
    }

    private boolean isArmAtTarget(double targetAngle) {
        double currentAngle = m_arm.getCurrentAngle();
        boolean atTarget = Math.abs(currentAngle - targetAngle) <= ARM_TOLERANCE;
        
        SmartDashboard.putNumber("ElevatorArmL4/CurrentAngle", currentAngle);
        SmartDashboard.putNumber("ElevatorArmL4/TargetAngle", targetAngle);
        SmartDashboard.putNumber("ElevatorArmL4/AngleError", Math.abs(currentAngle - targetAngle));
        
        return atTarget;
    }

    @Override
    public void execute() {
        SmartDashboard.putString("ElevatorArmL4/State", currentState.toString());
        
        switch (currentState) {
            case WAITING_FOR_DELAY:
                // Check if delay period is complete
                double currentTime = System.currentTimeMillis() / 1000.0;
                double elapsedTime = currentTime - startTime;
                
                SmartDashboard.putNumber("ElevatorArmL4/DelayTimeElapsed", elapsedTime);
                
                if (elapsedTime >= DELAY_SECONDS) {
                   // System.out.println("ElevatorArmL4: Delay complete, now setting elevator height");
                    m_elevator.setHeight(SafetyConstants.L4[0]);
                    m_arm.setAngle(SafetyConstants.L4[1]);
                    m_coralIntake.setIntakeSpeed(.06);
                    currentState = SetupState.ELEVATOR_UP_ARM_TO_SCORE;
                }
                break;
                
            case ELEVATOR_UP_ARM_TO_SCORE:
                // Wait for elevator to reach L4 height
                if (isElevatorAtTarget()&&isArmAtTarget(SafetyConstants.L4[1])) {
                    //System.out.println("ElevatorArmL4: Elevator at height, moving arm");
                    currentState = SetupState.READY_TO_SCORE;
                    m_arm.setAngle(SafetyConstants.L4[1]);
                    m_elevator.setHeight(SafetyConstants.L4[0]);
                    m_coralIntake.setIntakeSpeed(.06);
                    currentState = SetupState.READY_TO_SCORE;
                }
                break;
                
            case READY_TO_SCORE:
                // Maintain position, waiting for coral ejection command
                // This state doesn't progress further - another command will handle the coral ejection
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // Command never finishes on its own - it will maintain the position
        // until another command takes control of the subsystems
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("ElevatorArmL4: Command interrupted");
        }
        // No need to reset positions, as other commands will control the subsystems after this
    }
}