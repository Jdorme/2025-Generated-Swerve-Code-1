package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SafetyConstants;
import frc.robot.subsystems.SafetySubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgaeIntake;

public class ArmClimbPositionCommand extends Command {
    private enum IntakeState {
        MOVE_TO_STOW,      // First move to stowed position using safety subsystem
        WAIT_FOR_STOW,     // Wait until completely at stowed position
        ARM_TO_FLOOR,      // Then move arm to floor position
        ELEVATOR_TO_FLOOR, // Finally move elevator to floor position
        DONE               // Command complete
    }

    private final SafetySubsystem m_safetySystem;
    private final ArmSubsystem m_arm;
    private final ElevatorSubsystem m_elevator;
    private final AlgaeIntake m_algaeIntake;
    private IntakeState currentState = IntakeState.MOVE_TO_STOW;
    private boolean wasLastStateBallPresent = false;
    
    private static final double ELEVATOR_TOLERANCE = 1; // inches
    private static final double ARM_TOLERANCE = 10; // degrees
    
    public ArmClimbPositionCommand(SafetySubsystem safetySystem, ArmSubsystem arm, 
                                   ElevatorSubsystem elevator, AlgaeIntake algaeIntake) {
        m_safetySystem = safetySystem;
        m_arm = arm;
        m_elevator = elevator;
        m_algaeIntake = algaeIntake;
        addRequirements(safetySystem, algaeIntake);
    }

    @Override
    public void initialize() {
        System.out.println("ClimbPosition: Starting command - using safety system to move to stow first");
        currentState = IntakeState.MOVE_TO_STOW;
       // wasLastStateBallPresent = false;
        
        // First step: Move to stowed position using the safety subsystem
        // This ensures proper sequencing when coming from a low position
        m_safetySystem.setTargetPosition(
            SafetyConstants.STOWED[0], 
            SafetyConstants.STOWED[1]
        );
    }

    private boolean isElevatorAtTarget(double targetHeight) {
        double currentHeight = m_elevator.getCurrentHeight();
        boolean atTarget = Math.abs(currentHeight - targetHeight) <= ELEVATOR_TOLERANCE;
        
        SmartDashboard.putNumber("ClimbPosition/CurrentHeight", currentHeight);
        SmartDashboard.putNumber("ClimbPosition/TargetHeight", targetHeight);
        SmartDashboard.putNumber("ClimbPosition/HeightError", Math.abs(currentHeight - targetHeight));
        
        return atTarget;
    }

    private boolean isArmAtTarget(double targetAngle) {
        double currentAngle = m_arm.getCurrentAngle();
        boolean atTarget = Math.abs(currentAngle - targetAngle) <= ARM_TOLERANCE;
        
        SmartDashboard.putNumber("ClimbPosition/CurrentAngle", currentAngle);
        SmartDashboard.putNumber("ClimbPosition/TargetAngle", targetAngle);
        SmartDashboard.putNumber("ClimbPosition/AngleError", Math.abs(currentAngle - targetAngle));
        
        return atTarget;
    }
    
    private boolean areBothAtTarget(double targetHeight, double targetAngle) {
        boolean elevatorReady = isElevatorAtTarget(targetHeight);
        boolean armReady = isArmAtTarget(targetAngle);
        
        SmartDashboard.putBoolean("ClimbPosition/ElevatorReady", elevatorReady);
        SmartDashboard.putBoolean("ClimbPosition/ArmReady", armReady);
        
        return elevatorReady && armReady;
    }

    @Override
    public void execute() {
        SmartDashboard.putString("ClimbPosition/State", currentState.toString());

        switch (currentState) {
            case MOVE_TO_STOW:
                // First ensure we're at a safe height using the safety subsystem
                if (m_safetySystem.isAtTarget()) {
                    System.out.println("ClimbPosition: Safety system has reached stowed position");
                    currentState = IntakeState.WAIT_FOR_STOW;
                }
                break;
                
            case WAIT_FOR_STOW:
                // Double-check that both arm and elevator are actually at the stowed position
                if (areBothAtTarget(SafetyConstants.STOWED[0], SafetyConstants.STOWED[1])) {
                    System.out.println("ClimbPosition: Confirmed at stowed position, moving arm to climb position");
                    currentState = IntakeState.ARM_TO_FLOOR;
                    // Now move arm to floor position - safe to do directly now that we're at stow
                    m_arm.setAngle(SafetyConstants.CLIMB_POSITION[1]);
                }
                break;
                
            case ARM_TO_FLOOR:
                // Wait for arm to reach floor position
                if (isArmAtTarget(SafetyConstants.CLIMB_POSITION[1])) {
                    System.out.println("ClimbPosition: Arm at climb angle, moving elevator to climb position");
                    currentState = IntakeState.ELEVATOR_TO_FLOOR;
                    // Move elevator to floor position
                    m_elevator.setHeight(SafetyConstants.CLIMB_POSITION[0]);
                }
                break;
                
            case ELEVATOR_TO_FLOOR:
                // Wait for elevator to reach floor position
                if (isElevatorAtTarget(SafetyConstants.CLIMB_POSITION[0])) {
                    System.out.println("ClimbPosition: Reached climb position, continuing to maintain position");
                    currentState = IntakeState.DONE;
                }
                break;

            case DONE:
                // Keep setting the target positions to maintain them
                m_arm.setAngle(SafetyConstants.CLIMB_POSITION[1]);
                m_elevator.setHeight(SafetyConstants.CLIMB_POSITION[0]);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // Never finish - keep running until interrupted to prevent other commands
        // from taking over and changing the position
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("ClimbPosition: Command interrupted");
        } else {
            System.out.println("ClimbPosition: Command completed normally");
        }
        
        // Return to stowed position using safety system
        m_safetySystem.setTargetPosition(
            SafetyConstants.STOWED[0], 
            SafetyConstants.STOWED[1]
        );
    }
}