package frc.robot.Commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SafetyConstants;
import frc.robot.subsystems.SafetySubsystem;
import frc.robot.subsystems.AlgaeIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L3AlgaeCommand extends Command {
  private final SafetySubsystem m_safetySystem;
  private final AlgaeIntake m_algaeIntake;
  private boolean wasLastStateBallPresent = false;
  
  public L3AlgaeCommand(SafetySubsystem safetySystem, AlgaeIntake algaeIntake) {
      m_safetySystem = safetySystem;
      m_algaeIntake = algaeIntake;
      addRequirements(safetySystem, algaeIntake);
  }

  @Override
  public void initialize() {
      // Reset state tracking
      wasLastStateBallPresent = false;
      
      // Move to L3 algae position using safety system
      m_safetySystem.setTargetPosition(
          SafetyConstants.L3_ALGAE[0], 
          SafetyConstants.L3_ALGAE[1]
      );
  }

  @Override
  public void execute() {
      boolean hasBall = m_algaeIntake.hasBall();
      
      // If we're in position and don't have a ball yet, run intake
      if (m_safetySystem.isAtTarget() && !hasBall) {
          m_algaeIntake.intake();
      }
      
      // If we just got a ball, output to console for debugging
      if (hasBall && !wasLastStateBallPresent) {
          System.out.println("L3Algae: Ball acquired, holding position");
      }
      
      // Update last state
      wasLastStateBallPresent = hasBall;

      // Log state to SmartDashboard
      SmartDashboard.putBoolean("L3Algae/AtPosition", m_safetySystem.isAtTarget());
      SmartDashboard.putBoolean("L3Algae/HasBall", hasBall);
      SmartDashboard.putString("L3Algae/State", hasBall ? "HOLDING" : "INTAKING");
  }

  @Override
  public void end(boolean interrupted) {
      if (interrupted) {
          System.out.println("L3Algae: Command interrupted");
      }
      
      // If we don't have a ball, stop the intake
      if (!m_algaeIntake.hasBall()) {
          System.out.println("L3Algae: No ball held, stopping intake");
          m_algaeIntake.stop();
      } else {
          System.out.println("L3Algae: Ball held, maintaining hold mode");
          // The AlgaeIntake will automatically maintain hold mode
      }
      
      // Return to stowed position
      System.out.println("L3Algae: Returning to stowed position");
 
  }

  @Override
  public boolean isFinished() {
      return false;  // Runs until button is released
  }
}