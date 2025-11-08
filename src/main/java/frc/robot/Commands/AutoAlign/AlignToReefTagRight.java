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
 * Aligns the robot to the RIGHT side of the reef using the Climber Limelight.
 * Uses limelight-climber for vision tracking.
 */
public class AlignToReefTagRight extends Command {
  private final CommandSwerveDrivetrain drivebase;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotController;
  
  private final Timer dontSeeTagTimer;
  private final Timer stopTimer;
  
  private double tagID = -1;
  
  // Limelight name for right side scoring
  private static final String LIMELIGHT_NAME = "limelight-climber";
  
  // SwerveRequest for robot-oriented driving (relative to robot's front)
  private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
      .withDeadband(0.0)
      .withRotationalDeadband(0.0);

  /**
   * Creates a new AlignToReefTagRight command for RIGHT side scoring.
   * Uses robot-centric control for precise vision-based alignment.
   * 
   * @param drivebase The swerve drivetrain subsystem
   */
  public AlignToReefTagRight(CommandSwerveDrivetrain drivebase) {
    this.drivebase = drivebase;
    
    // Initialize PID controllers
    xController = new PIDController(Constants.ReefAlignConstants.X_REEF_ALIGNMENT_P, 0.0, 0.0);
    yController = new PIDController(Constants.ReefAlignConstants.Y_REEF_ALIGNMENT_P, 0.0, 0.0);
    rotController = new PIDController(Constants.ReefAlignConstants.ROT_REEF_ALIGNMENT_P, 0.0, 0.0);
    
    // Initialize timers
    dontSeeTagTimer = new Timer();
    stopTimer = new Timer();
    
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    // Reset and start timers
    stopTimer.reset();
    stopTimer.start();
    dontSeeTagTimer.reset();
    dontSeeTagTimer.start();
    
    // Configure rotation controller
    rotController.setSetpoint(Constants.ReefAlignConstants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.ReefAlignConstants.ROT_TOLERANCE_REEF_ALIGNMENT);
    rotController.enableContinuousInput(-180, 180);  // Enable continuous input for rotation
    
    // Configure X (forward/backward) controller
    xController.setSetpoint(Constants.ReefAlignConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.ReefAlignConstants.X_TOLERANCE_REEF_ALIGNMENT);
    
    // Configure Y (left/right) controller - RIGHT side uses negative setpoint
    yController.setSetpoint(Constants.ReefAlignConstants.rightY_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(Constants.ReefAlignConstants.Y_TOLERANCE_REEF_ALIGNMENT);
    
    // Store the initial tag ID to track
    tagID = LimelightHelpers.getFiducialID(LIMELIGHT_NAME);
    
    SmartDashboard.putString("Reef Align Status", "Initializing RIGHT");
    SmartDashboard.putNumber("Target Tag ID", tagID);
    SmartDashboard.putString("Active Limelight", LIMELIGHT_NAME);
  }

  @Override
  public void execute() {
    // Check if we can see the target tag
    boolean hasValidTarget = LimelightHelpers.getTV(LIMELIGHT_NAME) 
        && LimelightHelpers.getFiducialID(LIMELIGHT_NAME) == tagID;
    
    if (hasValidTarget) {
      // Reset the "don't see tag" timer since we have a valid target
      dontSeeTagTimer.reset();
      
      // Get robot position relative to the AprilTag
      // Index meanings from target space:
      // [0] = X (side-to-side relative to tag)
      // [1] = Y (up-down, typically not used for swerve)
      // [2] = Z (distance from tag)
      // [3] = Roll
      // [4] = Pitch  
      // [5] = Yaw (rotation relative to tag)
      double[] positions = LimelightHelpers.getBotPose_TargetSpace(LIMELIGHT_NAME);
      
      // Calculate control outputs
      // X Speed: Forward/backward movement (based on distance from tag)
      double xSpeed = xController.calculate(positions[2]);
      
      // Y Speed: Left/right strafe movement (based on horizontal offset)
      double ySpeed = -yController.calculate(positions[0]);
      
      // Rotation: Turn to face tag properly (based on yaw angle to tag)
      double rotValue = -rotController.calculate(positions[4]);
      
      // Apply the calculated speeds to the drivetrain (robot-centric)
      drivebase.setControl(robotCentricRequest
          .withVelocityX(xSpeed)
          .withVelocityY(ySpeed)
          .withRotationalRate(rotValue));
      
      // Check if all controllers are at setpoint
      boolean atTarget = rotController.atSetpoint() 
          && yController.atSetpoint() 
          && xController.atSetpoint();
      
      // Reset stop timer if we're not at target
      if (!atTarget) {
        stopTimer.reset();
      }
      
      // Debug output
      SmartDashboard.putNumber("Reef RIGHT X Position", positions[2]);
      SmartDashboard.putNumber("Reef RIGHT Y Position", positions[0]);
      SmartDashboard.putNumber("Reef RIGHT Yaw", positions[4]);
      SmartDashboard.putNumber("Reef RIGHT X Speed", xSpeed);
      SmartDashboard.putNumber("Reef RIGHT Y Speed", ySpeed);
      SmartDashboard.putNumber("Reef RIGHT Rot Speed", rotValue);
      SmartDashboard.putBoolean("Reef RIGHT At Target", atTarget);
      SmartDashboard.putString("Reef Align Status", "Aligning RIGHT");
      
    } else {
      // No valid target - stop the robot
      drivebase.setControl(robotCentricRequest
          .withVelocityX(0)
          .withVelocityY(0)
          .withRotationalRate(0));
      
      SmartDashboard.putString("Reef Align Status", "No Target RIGHT");
    }
    
    SmartDashboard.putNumber("Reef RIGHT Stop Timer", stopTimer.get());
    SmartDashboard.putNumber("Reef RIGHT Lost Target Timer", dontSeeTagTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    drivebase.setControl(robotCentricRequest
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0));
    
    SmartDashboard.putString("Reef Align Status", interrupted ? "Interrupted RIGHT" : "Completed RIGHT");
  }

  @Override
  public boolean isFinished() {
    // Finish if:
    // 1. We haven't seen the tag for too long (lost target), OR
    // 2. We've been at the correct position for the required validation time
    boolean lostTarget = dontSeeTagTimer.hasElapsed(Constants.ReefAlignConstants.DONT_SEE_TAG_WAIT_TIME);
    boolean atTargetLongEnough = stopTimer.hasElapsed(Constants.ReefAlignConstants.POSE_VALIDATION_TIME);
    
    return lostTarget || atTargetLongEnough;
  }
}