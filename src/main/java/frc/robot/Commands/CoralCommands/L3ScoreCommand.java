package frc.robot.Commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SafetyConstants;
import frc.robot.subsystems.SafetySubsystem;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class L3ScoreCommand extends Command {
    private enum ScoreState {
        ELEVATOR_UP,
        ARM_TO_SCORE,
        SCORING,
        ARM_BACK,
        ELEVATOR_STOW,
        DONE
    }

    private final SafetySubsystem m_safetySystem;
    private final CoralIntake m_coralIntake;
    private final ElevatorSubsystem m_elevator;
    private final ArmSubsystem m_arm;
    private ScoreState currentState = ScoreState.ELEVATOR_UP;
    private double waitStartTime = 0;
    
    private static final double ELEVATOR_TOLERANCE = 0.5;
    private static final double ARM_TOLERANCE = 2.0;
    private static final double SCORING_TIME = .125;
    
    public L3ScoreCommand(SafetySubsystem safetySystem, CoralIntake coralIntake, 
                         ElevatorSubsystem elevator, ArmSubsystem arm) {
        m_safetySystem = safetySystem;
        m_coralIntake = coralIntake;
        m_elevator = elevator;
        m_arm = arm;
        addRequirements(safetySystem, coralIntake);
    }

    @Override
    public void initialize() {
        System.out.println("L3Score: Starting command");
        currentState = ScoreState.ELEVATOR_UP;
        m_elevator.setHeight(SafetyConstants.L3[0]);
        m_arm.setAngle(SafetyConstants.L3[1]);
    }

    private boolean isElevatorAtTarget() {
        double currentHeight = m_elevator.getCurrentHeight();
        double targetHeight = SafetyConstants.L3[0];
        boolean atTarget = Math.abs(currentHeight - targetHeight) <= ELEVATOR_TOLERANCE;
        
        SmartDashboard.putNumber("L3Score/CurrentHeight", currentHeight);
        SmartDashboard.putNumber("L3Score/TargetHeight", targetHeight);
        SmartDashboard.putNumber("L3Score/HeightError", Math.abs(currentHeight - targetHeight));
        
        return atTarget;
    }

    private boolean isArmAtTarget(double targetAngle) {
        double currentAngle = m_arm.getCurrentAngle();
        boolean atTarget = Math.abs(currentAngle - targetAngle) <= ARM_TOLERANCE;
        
        SmartDashboard.putNumber("L3Score/CurrentAngle", currentAngle);
        SmartDashboard.putNumber("L3Score/TargetAngle", targetAngle);
        SmartDashboard.putNumber("L3Score/AngleError", Math.abs(currentAngle - targetAngle));
        
        return atTarget;
    }

    @Override
    public void execute() {
        SmartDashboard.putString("L3Score/State", currentState.toString());

        switch (currentState) {
            case ELEVATOR_UP:
                if (isElevatorAtTarget()) {
                    System.out.println("L3Score: Elevator at height, moving arm");
                    currentState = ScoreState.ARM_TO_SCORE;
                    m_arm.setAngle(SafetyConstants.L3[1]);
                }
                break;

            case ARM_TO_SCORE:
                if (isArmAtTarget(SafetyConstants.L3[1])) {
                    System.out.println("L3Score: Arm at scoring angle, starting coral");
                    currentState = ScoreState.SCORING;
                    m_coralIntake.reverse();
                    waitStartTime = System.currentTimeMillis();
                }
                break;

            case SCORING:
                if ((System.currentTimeMillis() - waitStartTime) >= (SCORING_TIME * 1000)) {
                    System.out.println("L3Score: Moving arm back while continuing to score");
                    currentState = ScoreState.ARM_BACK;
                    m_arm.setAngle(0);
                }
                break;

            case ARM_BACK:
                if (isArmAtTarget(0)) {
                    System.out.println("L3Score: Arm back at zero, stopping coral and lowering elevator");
                    m_coralIntake.stop();
                    currentState = ScoreState.ELEVATOR_STOW;
                    m_elevator.setHeight(SafetyConstants.STOWED[0]);
                }
                break;

            case ELEVATOR_STOW:
                if (Math.abs(m_elevator.getCurrentHeight() - SafetyConstants.STOWED[0]) <= ELEVATOR_TOLERANCE) {
                    System.out.println("L3Score: At stowed position, complete");
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
            System.out.println("L3Score: Command interrupted");
            m_coralIntake.stop();
            m_arm.setAngle(0);
            m_elevator.setHeight(SafetyConstants.STOWED[0]);
        } else {
            System.out.println("L3Score: Command completed normally");
        }
    }
}