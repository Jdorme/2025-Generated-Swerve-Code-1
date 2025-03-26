package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SafetyConstants;
import frc.robot.subsystems.SafetySubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * SafeInitializationCommand - Ensures the robot mechanisms are in a safe state at the start of teleop
 * or after any unexpected power cycle.
 */
public class SafeInitializationCommand extends Command {
    private enum InitState {
        CHECK_POSITION,    // Determine the current state of mechanisms
        ELEVATOR_UP,       // Move elevator to safe height first if needed
        ARM_SAFE,          // Move arm to safe angle once elevator is high enough
        ARM_FIRST,         // New state: Rotate arm to 0 before moving elevator
        MOVE_TO_STOW,      // Finally move to fully stowed position
        DONE                // Command complete
    }

    private final SafetySubsystem m_safetySystem;
    private final ArmSubsystem m_arm;
    private final ElevatorSubsystem m_elevator;
    private InitState currentState = InitState.CHECK_POSITION;
    
    // Position safety thresholds
    private static final double DANGER_ZONE_HEIGHT = 5.9; // inches
    private static final double SAFE_HEIGHT = 6.0; // inches - minimum safe height
    private static final double MAX_SAFE_ELEVATOR_HEIGHT = 8.0; // inches - maximum height for arm adjustment
    private static final double ELEVATOR_TOLERANCE = 0.5; // inches
    private static final double ARM_TOLERANCE = 2.0; // degrees
    
    public SafeInitializationCommand(SafetySubsystem safetySystem, ArmSubsystem arm, 
                                   ElevatorSubsystem elevator) {
        m_safetySystem = safetySystem;
        m_arm = arm;
        m_elevator = elevator;
        addRequirements(safetySystem);
    }

    @Override
    public void initialize() {
        System.out.println("SafeInit: Starting safety initialization sequence");
        currentState = InitState.CHECK_POSITION;
    }

    private boolean isElevatorAtTarget(double targetHeight) {
        double currentHeight = m_elevator.getCurrentHeight();
        boolean atTarget = Math.abs(currentHeight - targetHeight) <= ELEVATOR_TOLERANCE;
        
        SmartDashboard.putNumber("SafeInit/CurrentHeight", currentHeight);
        SmartDashboard.putNumber("SafeInit/TargetHeight", targetHeight);
        
        return atTarget;
    }

    private boolean isArmAtTarget(double targetAngle) {
        double currentAngle = m_arm.getCurrentAngle();
        boolean atTarget = Math.abs(currentAngle - targetAngle) <= ARM_TOLERANCE;
        
        SmartDashboard.putNumber("SafeInit/CurrentAngle", currentAngle);
        SmartDashboard.putNumber("SafeInit/TargetAngle", targetAngle);
        
        return atTarget;
    }

    @Override
    public void execute() {
        double currentHeight = m_elevator.getCurrentHeight();
        double currentAngle = m_arm.getCurrentAngle();
        
        SmartDashboard.putString("SafeInit/State", currentState.toString());

        switch (currentState) {
            case CHECK_POSITION:
                // Determine what safety measures are needed based on current positions
                if (currentHeight < DANGER_ZONE_HEIGHT) {
                    // Elevator is too low - need to raise it first for safety
                    System.out.println("SafeInit: Elevator in danger zone, raising to safe height first");
                    m_elevator.setHeight(SAFE_HEIGHT);
                    currentState = InitState.ELEVATOR_UP;
                } else if (currentHeight > MAX_SAFE_ELEVATOR_HEIGHT && Math.abs(currentAngle) > 0) {
                    // Elevator is above 8 inches, but arm is not at 0 degrees, rotate arm first
                    System.out.println("SafeInit: Elevator is high, arm not at 0, rotating arm first");
                    currentState = InitState.ARM_FIRST;
                    m_arm.setAngle(0); // Rotate arm to 0 degrees
                } else if (Math.abs(currentAngle) > 45.0) {
                    // Arm is at a dangerous angle but elevator is high enough
                    System.out.println("SafeInit: Arm at dangerous angle, moving to safe position");
                    currentState = InitState.ARM_SAFE;
                    m_arm.setAngle(0); // Move arm to horizontal position
                } else {
                    // Both mechanisms are in relatively safe positions, move to stow
                    System.out.println("SafeInit: Mechanisms in safe positions, moving to stow");
                    currentState = InitState.MOVE_TO_STOW;
                    m_safetySystem.setTargetPosition(
                        SafetyConstants.STOWED[0], 
                        SafetyConstants.STOWED[1]
                    );
                }
                break;
            
            case ELEVATOR_UP:
                // Wait for elevator to reach safe height before doing anything with the arm
                if (isElevatorAtTarget(SAFE_HEIGHT)) {
                    System.out.println("SafeInit: Elevator at safe height, now addressing arm");
                    currentState = InitState.ARM_SAFE;
                    m_arm.setAngle(0); // Move arm to horizontal position
                } else {
                    // Elevator not at safe height yet, keep commanding it to the safe height
                    m_elevator.setHeight(SAFE_HEIGHT);
                }
                break;
                
            case ARM_FIRST:
                // Wait for arm to reach 0 degrees before doing anything with the elevator
                if (isArmAtTarget(0)) {
                    System.out.println("SafeInit: Arm at 0 degrees, now moving elevator");
                    m_elevator.setHeight(SAFE_HEIGHT); // Now we can safely move the elevator
                    currentState = InitState.ELEVATOR_UP;
                }
                break;

            case ARM_SAFE:
                // Make sure elevator stays at the safe height
                if (currentHeight < SAFE_HEIGHT) {
                    System.out.println("SafeInit: Elevator dropped below safe height, raising again");
                    m_elevator.setHeight(SAFE_HEIGHT);
                }
                
                // Wait for arm to reach safe angle
                if (isArmAtTarget(0)) {
                    System.out.println("SafeInit: Arm at safe position, moving to stowed position");
                    currentState = InitState.MOVE_TO_STOW;
                    // Now use safety system to reach final stowed position
                    m_safetySystem.setTargetPosition(
                        SafetyConstants.STOWED[0], 
                        SafetyConstants.STOWED[1]
                    );
                }
                break;
            
            case MOVE_TO_STOW:
                // Wait for both systems to fully stow
                if (m_safetySystem.isAtTarget()) {
                    System.out.println("SafeInit: Reached stowed position, safety initialization complete");
                    currentState = InitState.DONE;
                }
                break;

            case DONE:
                // Just wait until command finishes
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return currentState == InitState.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("SafeInit: Safety initialization interrupted");
        } else {
            System.out.println("SafeInit: Safety initialization completed successfully");
        }
    }
}
