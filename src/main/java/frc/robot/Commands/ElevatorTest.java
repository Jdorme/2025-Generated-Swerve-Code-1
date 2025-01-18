// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorTest extends Command {
  private final ElevatorSubsystem m_elevator;
  private final double m_targetHeight;

  /** Creates a new ElevatorTest. */
  public ElevatorTest(ElevatorSubsystem elevator, double targetHeightInches) {
    m_elevator = elevator;
    m_targetHeight = targetHeightInches;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Moving elevator to height: " + m_targetHeight);
    m_elevator.setHeight(m_targetHeight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Log data to SmartDashboard for testing
    
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_elevator.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.isAtTarget();
  }
}