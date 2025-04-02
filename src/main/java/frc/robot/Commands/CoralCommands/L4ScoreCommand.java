package frc.robot.Commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SafetyConstants;
import frc.robot.subsystems.SafetySubsystem;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class L4ScoreCommand extends Command {
    private enum ScoreState {
        ELEVATOR_UP,           // Moving elevator to scoring height
        ARM_TO_SCORE,         // Moving arm to scoring angle
        SCORING,              // Running coral in reverse
        ARM_BACK,            // Moving arm back to zero
        ELEVATOR_STOW,        // Moving elevator to stowed position
        DONE                  // Complete
    }

    private final SafetySubsystem m_safetySystem;
    private final CoralIntake m_coralIntake;
    private final ElevatorSubsystem m_elevator;
    private final ArmSubsystem m_arm;
    private ScoreState currentState = ScoreState.ELEVATOR_UP;
    private double waitStartTime = 0;
    
    // Position tolerances
    private static final double ELEVATOR_TOLERANCE = 0.5; // inches
    private static final double ARM_TOLERANCE = 2.0; // degrees
    private static final double SCORING_TIME = .25; // seconds
    
    public L4ScoreCommand(SafetySubsystem safetySystem, CoralIntake coralIntake, 
                         ElevatorSubsystem elevator, ArmSubsystem arm) {
        m_safetySystem = safetySystem;
        m_coralIntake = coralIntake;
        m_elevator = elevator;
        m_arm = arm;
        // Add all subsystems as requirements to prevent command conflicts
        addRequirements(safetySystem, coralIntake, elevator, arm);
    }

    @Override
    public void initialize() {
        System.out.println("L4Score: Starting command");
        currentState = ScoreState.ELEVATOR_UP;
        // Start by moving elevator to L4 height
        m_elevator.setHeight(SafetyConstants.L4[0]);
        m_arm.setAngle(34); // Ensure arm is at zero while elevator moves
    }

    private boolean isElevatorAtTarget() {
        double currentHeight = m_elevator.getCurrentHeight();
        // Use the actual target height from constants instead of hardcoded value
        double targetHeight = SafetyConstants.L4[0];
        boolean atTarget = Math.abs(currentHeight - targetHeight) <= ELEVATOR_TOLERANCE;
        
        SmartDashboard.putNumber("L4Score/CurrentHeight", currentHeight);
        SmartDashboard.putNumber("L4Score/TargetHeight", targetHeight);
        SmartDashboard.putNumber("L4Score/HeightError", Math.abs(currentHeight - targetHeight));
        
        return atTarget;
    }

    private boolean isElevatorAtStowedTarget() {
        double currentHeight = m_elevator.getCurrentHeight();
        double targetHeight = SafetyConstants.STOWED[0];
        boolean atTarget = Math.abs(currentHeight - targetHeight) <= ELEVATOR_TOLERANCE;
        
        SmartDashboard.putNumber("L4Score/CurrentHeight", currentHeight);
        SmartDashboard.putNumber("L4Score/StowedHeight", targetHeight);
        SmartDashboard.putNumber("L4Score/HeightError", Math.abs(currentHeight - targetHeight));
        
        return atTarget;
    }

    private boolean isArmAtTarget(double targetAngle) {
        double currentAngle = m_arm.getCurrentAngle();
        boolean atTarget = Math.abs(currentAngle - targetAngle) <= ARM_TOLERANCE;
        
        SmartDashboard.putNumber("L4Score/CurrentAngle", currentAngle);
        SmartDashboard.putNumber("L4Score/TargetAngle", targetAngle);
        SmartDashboard.putNumber("L4Score/AngleError", Math.abs(currentAngle - targetAngle));
        
        return atTarget;
    }

    @Override
    public void execute() {
        SmartDashboard.putString("L4Score/State", currentState.toString());

        switch (currentState) {
            case ELEVATOR_UP:
                // Wait for elevator to reach L4 height
                if (isElevatorAtTarget()) {
                    System.out.println("L4Score: Elevator at height, moving arm");
                    currentState = ScoreState.ARM_TO_SCORE;
                    m_arm.setAngle(SafetyConstants.L4[1]);
                }
                break;

            case ARM_TO_SCORE:
                // Wait for arm to reach scoring angle
                if (isArmAtTarget(SafetyConstants.L4[1])) {
                    System.out.println("L4Score: Arm at scoring angle, starting coral");
                    currentState = ScoreState.SCORING;
                    m_coralIntake.reverse();
                    waitStartTime = System.currentTimeMillis();
                }
                break;

            case SCORING:
                // Wait for scoring time
                if ((System.currentTimeMillis() - waitStartTime) >= (SCORING_TIME * 1000)) {
                    System.out.println("L4Score: Scoring complete, moving arm back");
                    currentState = ScoreState.ARM_BACK;
                    // Stop the coral intake immediately after scoring
                    m_coralIntake.stop();
                    m_arm.setAngle(0);
                }
                break;

            case ARM_BACK:
                // Wait for arm to return to zero
                if (isArmAtTarget(0)) {
                    System.out.println("L4Score: Arm at zero, lowering elevator");
                    // Intake already stopped in SCORING state
                    currentState = ScoreState.ELEVATOR_STOW;
                    m_elevator.setHeight(SafetyConstants.STOWED[0]);
                }
                break;

            case ELEVATOR_STOW:
                // Wait for elevator to reach stowed height
                if (isElevatorAtStowedTarget()) {
                    System.out.println("L4Score: At stowed position, complete");
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
            System.out.println("L4Score: Command interrupted");
            m_coralIntake.stop();
            // Safe return to stowed
            m_arm.setAngle(0);
            m_elevator.setHeight(SafetyConstants.STOWED[0]);
        } else {
            System.out.println("L4Score: Command completed normally");
        }
    }
}