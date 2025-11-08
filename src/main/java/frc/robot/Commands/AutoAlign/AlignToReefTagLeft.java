// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoAlign;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * SIMPLIFIED: Aligns using TX (horizontal crosshair offset) only
 */
public class AlignToReefTagLeft extends Command {
  private final CommandSwerveDrivetrain drivebase;
  private final PIDController txController;
  
  private final Timer dontSeeTagTimer;
  private final Timer stopTimer;
  
  private double tagID = -1;
  
  private static final String LIMELIGHT_NAME = "limelight-elevato";
  
  // Target TX value (degrees)
  private static final double TX_SETPOINT = 0.45;
  private static final double TX_TOLERANCE = 0.5;  // Adjust as needed
  
  private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
      .withDeadband(0.0)
      .withRotationalDeadband(0.0);

  public AlignToReefTagLeft(CommandSwerveDrivetrain drivebase) {
    this.drivebase = drivebase;
    
    // Controller for TX alignment
    txController = new PIDController(Constants.ReefAlignConstants.Y_REEF_ALIGNMENT_P, 0.0, 0.0);
    
    dontSeeTagTimer = new Timer();
    stopTimer = new Timer();
    
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    stopTimer.reset();
    stopTimer.start();
    dontSeeTagTimer.reset();
    dontSeeTagTimer.start();
    
    // Configure TX controller
    txController.setSetpoint(TX_SETPOINT);
    txController.setTolerance(TX_TOLERANCE);
    
    tagID = LimelightHelpers.getFiducialID(LIMELIGHT_NAME);
    
    SmartDashboard.putString("Reef Status", "TX-ONLY Mode Started");
    SmartDashboard.putNumber("Target Tag ID", tagID);
    SmartDashboard.putNumber("TX Setpoint", TX_SETPOINT);
  }

  @Override
  public void execute() {
    boolean hasValidTarget = LimelightHelpers.getTV(LIMELIGHT_NAME) 
        && LimelightHelpers.getFiducialID(LIMELIGHT_NAME) == tagID;
    
    if (hasValidTarget) {
      dontSeeTagTimer.reset();
      
      // Get TX (horizontal crosshair offset in degrees)
      double tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
      
      // Calculate strafe speed based on TX
      // Positive TX = target is to the right, so strafe right (positive Y)
      double ySpeed = txController.calculate(tx);
      double txError = TX_SETPOINT - tx;
      
      // Move ONLY left/right (strafe) based on TX - NO forward/back, NO rotation
      drivebase.setControl(robotCentricRequest
          .withVelocityX(0.0)  // LOCKED - no forward/back
          .withVelocityY(ySpeed)  // Strafe left/right
          .withRotationalRate(0.0));  // LOCKED - no rotation
      
      boolean atTarget = txController.atSetpoint();
      
      if (!atTarget) {
        stopTimer.reset();
      }
      
      // Debug output - SIMPLIFIED
      SmartDashboard.putNumber("TX Current (deg)", tx);
      SmartDashboard.putNumber("TX Setpoint (deg)", TX_SETPOINT);
      SmartDashboard.putNumber("TX Error (deg)", txError);
      SmartDashboard.putNumber("Y Speed Output (strafe)", ySpeed);
      SmartDashboard.putBoolean("TX At Target", atTarget);
      SmartDashboard.putNumber("TX P Gain", Constants.ReefAlignConstants.Y_REEF_ALIGNMENT_P);
      SmartDashboard.putString("Reef Status", "Aligning TX");
      
    } else {
      // No target - stop
      drivebase.setControl(robotCentricRequest
          .withVelocityX(0)
          .withVelocityY(0)
          .withRotationalRate(0));
      
      SmartDashboard.putString("Reef Status", "No Target");
    }
    
    SmartDashboard.putNumber("Stop Timer", stopTimer.get());
    SmartDashboard.putNumber("Lost Target Timer", dontSeeTagTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.setControl(robotCentricRequest
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0));
    
    SmartDashboard.putString("Reef Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    boolean lostTarget = dontSeeTagTimer.hasElapsed(Constants.ReefAlignConstants.DONT_SEE_TAG_WAIT_TIME);
    boolean atTargetLongEnough = stopTimer.hasElapsed(Constants.ReefAlignConstants.POSE_VALIDATION_TIME);
    
    return lostTarget || atTargetLongEnough;
  }
}