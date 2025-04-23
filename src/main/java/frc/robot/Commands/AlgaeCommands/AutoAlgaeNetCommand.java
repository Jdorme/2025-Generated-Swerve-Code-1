package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.SafetyConstants;
import frc.robot.subsystems.SafetySubsystem;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoAlgaeNetCommand extends Command {
    private enum ScoreState {
        ELEVATOR_UP,           // Moving elevator to scoring height
        ARM_TO_SCORE,          // Moving arm to scoring angle
        SCORING,               // Running algae in reverse
        ARM_BACK,              // Moving arm back to zero
        DONE                   // Complete
    }

  //  private final SafetySubsystem m_safetySystem;
    private final AlgaeIntake m_algaeIntake;
    private final ElevatorSubsystem m_elevator;
    private final ArmSubsystem m_arm;
    private ScoreState currentState = ScoreState.ELEVATOR_UP;
    private double waitStartTime = 0;
    
    // Position tolerances
    private static final double ELEVATOR_TOLERANCE = 0.5; // inches
    private static final double ARM_TOLERANCE = 2.0; // degrees
    private static final double SCORING_TIME = .75; // seconds
    
    public AutoAlgaeNetCommand(SafetySubsystem safetySystem, AlgaeIntake algaeIntake, 
                         ElevatorSubsystem elevator, ArmSubsystem arm) {
     //   m_safetySystem = safetySystem;
        m_algaeIntake = algaeIntake;
        m_elevator = elevator;
        m_arm = arm;
        addRequirements(algaeIntake, elevator, arm);

    }

    @Override
    public void initialize() {
        System.out.println("AutoNetScore: Starting command");
        currentState = ScoreState.ELEVATOR_UP;
        // Start by moving elevator to Net height
        m_elevator.setHeight(SafetyConstants.NET_ALGAE[0]);
    }

    private boolean isElevatorAtTarget() {
        double currentHeight = m_elevator.getCurrentHeight();
        double targetHeight = SafetyConstants.NET_ALGAE[0];
        boolean atTarget = Math.abs(currentHeight - targetHeight) <= ELEVATOR_TOLERANCE;
        
        SmartDashboard.putNumber("AutoNetScore/CurrentHeight", currentHeight);
        SmartDashboard.putNumber("AutoNetScore/TargetHeight", targetHeight);
        SmartDashboard.putNumber("AutoNetScore/HeightError", Math.abs(currentHeight - targetHeight));
        
        return atTarget;
    }

    private boolean isArmAtTarget(double targetAngle) {
        double currentAngle = m_arm.getCurrentAngle();
        boolean atTarget = Math.abs(currentAngle - targetAngle) <= ARM_TOLERANCE;
        
        SmartDashboard.putNumber("AutoNetScore/CurrentAngle", currentAngle);
        SmartDashboard.putNumber("AutoNetScore/TargetAngle", targetAngle);
        SmartDashboard.putNumber("AutoNetScore/AngleError", Math.abs(currentAngle - targetAngle));
        
        return atTarget;
    }

    @Override
    public void execute() {
        SmartDashboard.putString("AutoNetScore/State", currentState.toString());

        switch (currentState) {
            case ELEVATOR_UP:
                // Wait for elevator to reach Net height
                if (isElevatorAtTarget()) {
                    System.out.println("AutoNetScore: Elevator at height, moving arm");
                    currentState = ScoreState.ARM_TO_SCORE;
                    
                }
                break;

            case ARM_TO_SCORE:
                // Wait for arm to reach scoring angle
                m_arm.setAngle(SafetyConstants.NET_ALGAE[1]);
                if (isArmAtTarget(SafetyConstants.NET_ALGAE[1])) {
                    System.out.println("AutoNetScore: Arm at scoring angle, starting algae");
                    currentState = ScoreState.SCORING;
                    m_algaeIntake.reverse();
                    waitStartTime = System.currentTimeMillis();
                }
                break;

                case SCORING:
                // Wait for scoring time
                if ((System.currentTimeMillis() - waitStartTime) >= (SCORING_TIME * 1000)) {
                    System.out.println("AutoNetScore: Scoring complete, moving arm back");
                    currentState = ScoreState.ARM_BACK;
                    // Do NOT stop the motor here
                }
                break;
            
                case ARM_BACK:
                // Wait for arm to fully return before moving the elevator down
                
                    System.out.println("AutoNetScore: Arm returned to zero, lowering elevator");
                    m_algaeIntake.stop(); // Stop intake before moving the elevator
                    m_elevator.setHeight(SafetyConstants.STOWED[0]);
                    m_arm.setAngle(SafetyConstants.STOWED[1]); // Start lowering elevator
                    currentState = ScoreState.DONE; // Mark command as finished without waiting for elevator
                
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
            System.out.println("AutoNetScore: Command interrupted");
            m_algaeIntake.stop();
            // Safe return to stowed
            m_arm.setAngle(0);
            m_elevator.setHeight(SafetyConstants.STOWED[0]);
        } else {
            System.out.println("AutoNetScore: Command completed normally");
        }
    }
}