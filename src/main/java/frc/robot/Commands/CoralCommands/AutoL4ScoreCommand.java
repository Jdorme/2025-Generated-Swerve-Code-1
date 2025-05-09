package frc.robot.Commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.SafetyConstants;
import frc.robot.subsystems.SafetySubsystem;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoL4ScoreCommand extends Command {
    private enum ScoreState {
        ELEVATOR_UP,           // Moving elevator to scoring height
        ARM_TO_SCORE,          // Moving arm to scoring angle
        SCORING,               // Running coral in reverse
        ARM_BACK,              // Moving arm back to zero
        DONE                   // Complete
    }

  //  private final SafetySubsystem m_safetySystem;
    private final CoralIntake m_coralIntake;
    private final ElevatorSubsystem m_elevator;
    private final ArmSubsystem m_arm;
    private ScoreState currentState = ScoreState.ELEVATOR_UP;
    private double waitStartTime = 0;
    
    // Position tolerances
    private static final double ELEVATOR_TOLERANCE = 0.5; // inches
    private static final double ARM_TOLERANCE = 2.0; // degrees
    private static final double SCORING_TIME = .125; // seconds
    
    public AutoL4ScoreCommand(SafetySubsystem safetySystem, CoralIntake coralIntake, 
                         ElevatorSubsystem elevator, ArmSubsystem arm) {
     //   m_safetySystem = safetySystem;
        m_coralIntake = coralIntake;
        m_elevator = elevator;
        m_arm = arm;
        addRequirements(coralIntake, elevator, arm);

    }

    @Override
    public void initialize() {
        System.out.println("AutoL4Score: Starting command");
        currentState = ScoreState.ELEVATOR_UP;
        // Start by moving elevator to L4 height
        m_elevator.setHeight(SafetyConstants.L4[0]);
        m_arm.setAngle(0); // Ensure arm is at zero while elevator moves
    }

    private boolean isElevatorAtTarget() {
        double currentHeight = m_elevator.getCurrentHeight();
        double targetHeight = SafetyConstants.L4[0];
        boolean atTarget = Math.abs(currentHeight - targetHeight) <= ELEVATOR_TOLERANCE;
        
        SmartDashboard.putNumber("AutoL4Score/CurrentHeight", currentHeight);
        SmartDashboard.putNumber("AutoL4Score/TargetHeight", targetHeight);
        SmartDashboard.putNumber("AutoL4Score/HeightError", Math.abs(currentHeight - targetHeight));
        
        return atTarget;
    }

    private boolean isArmAtTarget(double targetAngle) {
        double currentAngle = m_arm.getCurrentAngle();
        boolean atTarget = Math.abs(currentAngle - targetAngle) <= ARM_TOLERANCE;
        
        SmartDashboard.putNumber("AutoL4Score/CurrentAngle", currentAngle);
        SmartDashboard.putNumber("AutoL4Score/TargetAngle", targetAngle);
        SmartDashboard.putNumber("AutoL4Score/AngleError", Math.abs(currentAngle - targetAngle));
        
        return atTarget;
    }

    @Override
    public void execute() {
        SmartDashboard.putString("AutoL4Score/State", currentState.toString());

        switch (currentState) {
            case ELEVATOR_UP:
                // Wait for elevator to reach L4 height
                if (isElevatorAtTarget()) {
                    System.out.println("AutoL4Score: Elevator at height, moving arm");
                    currentState = ScoreState.ARM_TO_SCORE;
                    
                }
                break;

            case ARM_TO_SCORE:
                // Wait for arm to reach scoring angle
                m_arm.setAngle(SafetyConstants.L4[1]);
                if (isArmAtTarget(SafetyConstants.L4[1])) {
                    System.out.println("AutoL4Score: Arm at scoring angle, starting coral");
                    currentState = ScoreState.SCORING;
                    m_coralIntake.reverse();
                    waitStartTime = System.currentTimeMillis();
                }
                break;

                case SCORING:
                // Wait for scoring time
                if ((System.currentTimeMillis() - waitStartTime) >= (SCORING_TIME * 1000)) {
                    System.out.println("AutoL4Score: Scoring complete, moving arm back");
                    currentState = ScoreState.ARM_BACK;
                    m_arm.setAngle(20);
                    // Do NOT stop the motor here
                }
                break;
            
                case ARM_BACK:
                // Wait for arm to fully return before moving the elevator down
                if (isArmAtTarget(20)) {
                    System.out.println("AutoL4Score: Arm returned to zero, lowering elevator");
                    m_coralIntake.stop(); // Stop intake before moving the elevator
                    m_elevator.setHeight(SafetyConstants.STOWED[0]);
                    m_arm.setAngle(SafetyConstants.STOWED[1]); // Start lowering elevator
                    currentState = ScoreState.DONE; // Mark command as finished without waiting for elevator
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
            System.out.println("AutoL4Score: Command interrupted");
            m_coralIntake.stop();
            // Safe return to stowed
            m_arm.setAngle(0);
            m_elevator.setHeight(SafetyConstants.STOWED[0]);
        } else {
            System.out.println("AutoL4Score: Command completed normally");
        }
    }
}