package frc.robot.Commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SafetyConstants;
import frc.robot.subsystems.SafetySubsystem;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class L2ScoreCommand extends Command {
    private enum ScoreState {
        ARM_START,         // Start arm movement
        ELEVATOR_DELAY,    // Short delay before starting elevator
        MOVE_TO_SCORE,     // Both arm and elevator moving to scoring position
        SCORING,           // Start scoring
        RETURN_TO_STOW,    // Return both systems to stowed position
        DONE
    }

    private final SafetySubsystem m_safetySystem;
    private final CoralIntake m_coralIntake;
    private final ElevatorSubsystem m_elevator;
    private final ArmSubsystem m_arm;
    private ScoreState currentState = ScoreState.ARM_START;
    private double waitStartTime = 0;
    private double elevatorDelayTime = 0;
    private static final double ELEVATOR_START_DELAY = 0.05; // seconds
    
    private static final double ELEVATOR_TOLERANCE = 0.5;
    private static final double ARM_TOLERANCE = 2.0;
    private static final double SCORING_TIME = .375;
    
    public L2ScoreCommand(SafetySubsystem safetySystem, CoralIntake coralIntake, 
                         ElevatorSubsystem elevator, ArmSubsystem arm) {
        m_safetySystem = safetySystem;
        m_coralIntake = coralIntake;
        m_elevator = elevator;
        m_arm = arm;
        addRequirements(safetySystem, coralIntake);
    }

    @Override
    public void initialize() {
        System.out.println("L2Score: Starting command - moving arm first");
        currentState = ScoreState.ARM_START;
        // Start arm movement first
        m_arm.setAngle(SafetyConstants.L2[1]);
        elevatorDelayTime = System.currentTimeMillis();
    }

    private boolean isElevatorAtTarget(double targetHeight) {
        double currentHeight = m_elevator.getCurrentHeight();
        boolean atTarget = Math.abs(currentHeight - targetHeight) <= ELEVATOR_TOLERANCE;
        
        SmartDashboard.putNumber("L2Score/CurrentHeight", currentHeight);
        SmartDashboard.putNumber("L2Score/TargetHeight", targetHeight);
        SmartDashboard.putNumber("L2Score/HeightError", Math.abs(currentHeight - targetHeight));
        
        return atTarget;
    }

    private boolean isArmAtTarget(double targetAngle) {
        double currentAngle = m_arm.getCurrentAngle();
        boolean atTarget = Math.abs(currentAngle - targetAngle) <= ARM_TOLERANCE;
        
        SmartDashboard.putNumber("L2Score/CurrentAngle", currentAngle);
        SmartDashboard.putNumber("L2Score/TargetAngle", targetAngle);
        SmartDashboard.putNumber("L2Score/AngleError", Math.abs(currentAngle - targetAngle));
        
        return atTarget;
    }
    
    private boolean areBothAtTarget(double targetHeight, double targetAngle) {
        boolean elevatorReady = isElevatorAtTarget(targetHeight);
        boolean armReady = isArmAtTarget(targetAngle);
        
        SmartDashboard.putBoolean("L2Score/ElevatorReady", elevatorReady);
        SmartDashboard.putBoolean("L2Score/ArmReady", armReady);
        
        return elevatorReady && armReady;
    }

    @Override
    public void execute() {
        SmartDashboard.putString("L2Score/State", currentState.toString());

        switch (currentState) {
            case ARM_START:
                // Start arm movement and wait 0.25 seconds before starting elevator
                if ((System.currentTimeMillis() - elevatorDelayTime) >= (ELEVATOR_START_DELAY * 1000)) {
                    System.out.println("L2Score: Starting elevator after delay");
                    currentState = ScoreState.ELEVATOR_DELAY;
                    m_elevator.setHeight(SafetyConstants.L2[0]);
                    currentState = ScoreState.MOVE_TO_SCORE;
                }
                break;
                
            case ELEVATOR_DELAY:
                // This state is just a placeholder - we transition immediately to MOVE_TO_SCORE
                // after setting the elevator height
                break;
                
            case MOVE_TO_SCORE:
                // Wait for both arm and elevator to reach target positions
                if (areBothAtTarget(SafetyConstants.L2[0], SafetyConstants.L2[1])) {
                    System.out.println("L2Score: Arm and elevator at scoring position, starting coral");
                    currentState = ScoreState.SCORING;
                    m_coralIntake.reverse();
                    waitStartTime = System.currentTimeMillis();
                }
                break;

            case SCORING:
                // Wait for scoring time
                if ((System.currentTimeMillis() - waitStartTime) >= (SCORING_TIME * 1000)) {
                    System.out.println("L2Score: Scoring complete, returning to stowed position");
                    m_coralIntake.stop();
                    currentState = ScoreState.RETURN_TO_STOW;
                    // Move both systems back simultaneously
                    m_elevator.setHeight(SafetyConstants.STOWED[0]);
                    m_arm.setAngle(0);
                }
                break;

            case RETURN_TO_STOW:
                // Wait for both systems to return to stowed positions
                if (areBothAtTarget(SafetyConstants.STOWED[0], 0)) {
                    System.out.println("L2Score: Returned to stowed position, command complete");
                    currentState = ScoreState.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return currentState == ScoreState.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("L2Score: Command interrupted");
            m_coralIntake.stop();
            m_arm.setAngle(0);
            m_elevator.setHeight(SafetyConstants.STOWED[0]);
        } else {
            System.out.println("L2Score: Command completed normally");
        }
    }
}