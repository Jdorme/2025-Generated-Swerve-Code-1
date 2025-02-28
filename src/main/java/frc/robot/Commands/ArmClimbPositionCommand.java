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
        MOVE_TO_STOW,      // First move to stowed position
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
    
    private static final double ELEVATOR_TOLERANCE = 0.5; // inches
    private static final double ARM_TOLERANCE = 2.0; // degrees
    
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
        System.out.println("FloorIntake: Starting command - moving to stow first");
        currentState = IntakeState.MOVE_TO_STOW;
        wasLastStateBallPresent = false;
        
        // First step: Move to stowed position
        m_elevator.setHeight(SafetyConstants.STOWED[0]);
        m_arm.setAngle(SafetyConstants.STOWED[1]);
    }

    private boolean isElevatorAtTarget(double targetHeight) {
        double currentHeight = m_elevator.getCurrentHeight();
        boolean atTarget = Math.abs(currentHeight - targetHeight) <= ELEVATOR_TOLERANCE;
        
        SmartDashboard.putNumber("FloorIntake/CurrentHeight", currentHeight);
        SmartDashboard.putNumber("FloorIntake/TargetHeight", targetHeight);
        SmartDashboard.putNumber("FloorIntake/HeightError", Math.abs(currentHeight - targetHeight));
        
        return atTarget;
    }

    private boolean isArmAtTarget(double targetAngle) {
        double currentAngle = m_arm.getCurrentAngle();
        boolean atTarget = Math.abs(currentAngle - targetAngle) <= ARM_TOLERANCE;
        
        SmartDashboard.putNumber("FloorIntake/CurrentAngle", currentAngle);
        SmartDashboard.putNumber("FloorIntake/TargetAngle", targetAngle);
        SmartDashboard.putNumber("FloorIntake/AngleError", Math.abs(currentAngle - targetAngle));
        
        return atTarget;
    }
    
    private boolean areBothAtTarget(double targetHeight, double targetAngle) {
        boolean elevatorReady = isElevatorAtTarget(targetHeight);
        boolean armReady = isArmAtTarget(targetAngle);
        
        SmartDashboard.putBoolean("FloorIntake/ElevatorReady", elevatorReady);
        SmartDashboard.putBoolean("FloorIntake/ArmReady", armReady);
        
        return elevatorReady && armReady;
    }

    @Override
    public void execute() {
        SmartDashboard.putString("FloorIntake/State", currentState.toString());

        switch (currentState) {
            case MOVE_TO_STOW:
                // Wait until we're at the stowed position
                if (areBothAtTarget(SafetyConstants.STOWED[0], SafetyConstants.STOWED[1])) {
                    System.out.println("FloorIntake: Reached stowed position, moving arm to floor position");
                    currentState = IntakeState.ARM_TO_FLOOR;
                    // Move arm to floor position
                    m_arm.setAngle(SafetyConstants.CLIMB_POSITION[1]);
                }
                break;
                
            case ARM_TO_FLOOR:
                // Wait for arm to reach floor position
                if (isArmAtTarget(SafetyConstants.CLIMB_POSITION[1])) {
                    System.out.println("FloorIntake: Arm at floor angle, moving elevator to floor position");
                    currentState = IntakeState.ELEVATOR_TO_FLOOR;
                    // Move elevator to floor position
                    m_elevator.setHeight(SafetyConstants.CLIMB_POSITION[0]);
                }
                break;
                
            case ELEVATOR_TO_FLOOR:
                // Wait for elevator to reach floor position
                if (isElevatorAtTarget(SafetyConstants.CLIMB_POSITION[0])) {
                    System.out.println("FloorIntake: Reached floor position, continuing to maintain position");
                    currentState = IntakeState.DONE;
                }
                break;

            case DONE:
                // Keep setting the target positions to maintain them
                m_arm.setAngle(SafetyConstants.CLIMB_POSITION[1]);
                m_elevator.setHeight(SafetyConstants.CLIMB_POSITION[0]);
                
                // L2AlgaeCommand intake behavior - start intake if we're in position and don't have a ball
                // boolean hasBall = m_algaeIntake.hasBall();
                
                // if (!hasBall) {
                //     m_algaeIntake.intake();
                // }
                
                // // If we just got a ball, output to console for debugging
                // if (hasBall && !wasLastStateBallPresent) {
                //     System.out.println("FloorIntake: Ball acquired, holding position");
                // }
                
                // // Update last state
                // wasLastStateBallPresent = hasBall;

                // // Log additional state information
                // SmartDashboard.putBoolean("FloorIntake/HasBall", hasBall);
                // SmartDashboard.putString("FloorIntake/IntakeState", hasBall ? "HOLDING" : "INTAKING");
                // break;
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
            System.out.println("FloorIntake: Command interrupted");
        } else {
            System.out.println("FloorIntake: Command completed normally");
        }
        
        // L2AlgaeCommand end behavior - stop intake or maintain hold
        if (!m_algaeIntake.hasBall()) {
            System.out.println("FloorIntake: No ball held, stopping intake");
            m_algaeIntake.stop();
        } else {
            System.out.println("FloorIntake: Ball held, maintaining hold mode");
            // The AlgaeIntake will automatically maintain hold mode
        }
        
        // Return to stowed position
        m_safetySystem.setTargetPosition(
            SafetyConstants.STOWED[0], 
            SafetyConstants.STOWED[1]
        );
    }
}