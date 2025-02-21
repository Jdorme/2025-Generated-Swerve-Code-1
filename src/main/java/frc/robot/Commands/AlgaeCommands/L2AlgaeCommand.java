package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SafetyConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class L2AlgaeCommand extends Command {
    private enum IntakeState {
        ELEVATOR_TO_L2,     // Moving elevator to L2 height
        ARM_TO_L2,          // Moving arm to L2 angle
        INTAKING,           // Running intake
        DONE                // Complete when button is released
    }

    private final AlgaeIntake m_algaeIntake;
    private final ElevatorSubsystem m_elevator;
    private final ArmSubsystem m_arm;
    private IntakeState currentState = IntakeState.ELEVATOR_TO_L2;
    private boolean hasStartedIntake = false;
    
    // Position tolerances
    private static final double ELEVATOR_TOLERANCE = 0.5; // inches
    private static final double ARM_TOLERANCE = 2.0; // degrees
    
    public L2AlgaeCommand(AlgaeIntake algaeIntake, ElevatorSubsystem elevator, ArmSubsystem arm) {
        m_algaeIntake = algaeIntake;
        m_elevator = elevator;
        m_arm = arm;
        addRequirements(algaeIntake);
    }

    @Override
    public void initialize() {
        System.out.println("L2Algae: Starting command");
        currentState = IntakeState.ELEVATOR_TO_L2;
        hasStartedIntake = false;
        
        // Start by moving elevator to L2 height
        m_elevator.setHeight(SafetyConstants.L2_ALGAE[0]);
        m_arm.setAngle(0); // Ensure arm is at zero while elevator moves
    }

    private boolean isElevatorAtTarget(double targetHeight) {
        double currentHeight = m_elevator.getCurrentHeight();
        boolean atTarget = Math.abs(currentHeight - targetHeight) <= ELEVATOR_TOLERANCE;
        
        SmartDashboard.putNumber("L2Algae/CurrentHeight", currentHeight);
        SmartDashboard.putNumber("L2Algae/TargetHeight", targetHeight);
        SmartDashboard.putNumber("L2Algae/HeightError", Math.abs(currentHeight - targetHeight));
        
        return atTarget;
    }

    private boolean isArmAtTarget(double targetAngle) {
        double currentAngle = m_arm.getCurrentAngle();
        boolean atTarget = Math.abs(currentAngle - targetAngle) <= ARM_TOLERANCE;
        
        SmartDashboard.putNumber("L2Algae/CurrentAngle", currentAngle);
        SmartDashboard.putNumber("L2Algae/TargetAngle", targetAngle);
        SmartDashboard.putNumber("L2Algae/AngleError", Math.abs(currentAngle - targetAngle));
        
        return atTarget;
    }

    @Override
    public void execute() {
        SmartDashboard.putString("L2Algae/State", currentState.toString());
        SmartDashboard.putBoolean("L2Algae/HasBall", m_algaeIntake.hasBall());

        switch (currentState) {
            case ELEVATOR_TO_L2:
                // Wait for elevator to reach L2 height
                if (isElevatorAtTarget(SafetyConstants.L2_ALGAE[0])) {
                    System.out.println("L2Algae: Elevator at height, moving arm");
                    currentState = IntakeState.ARM_TO_L2;
                    m_arm.setAngle(SafetyConstants.L2_ALGAE[1]);
                }
                break;

            case ARM_TO_L2:
                // Wait for arm to reach L2 angle
                if (isArmAtTarget(SafetyConstants.L2_ALGAE[1])) {
                    System.out.println("L2Algae: Arm at angle, starting intake");
                    currentState = IntakeState.INTAKING;
                    m_algaeIntake.intake();
                    hasStartedIntake = true;
                }
                break;

            case INTAKING:
                // Continue intaking until button is released
                // If we have a ball, we can display it but keep running
                if (m_algaeIntake.hasBall() && !hasStartedIntake) {
                    System.out.println("L2Algae: Ball detected!");
                }
                break;

            case DONE:
                // Should never reach here, as isFinished() is always false
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // Command never finishes on its own - runs until the button is released
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("L2Algae: Command ending, starting stow sequence");
        
        // Stop intake
        m_algaeIntake.stop();
        
        // Safe stow sequence - arm first, then elevator
        m_arm.setAngle(0);
        
        // The periodic scheduler will continue running after this command ends,
        // so we can safely assume the arm will be moved to zero. Once that's done,
        // another command or the robot's default state should handle stowing the elevator.
        m_elevator.setHeight(SafetyConstants.STOWED[0]);
        
        System.out.println("L2Algae: Stow sequence initiated");
    }
}