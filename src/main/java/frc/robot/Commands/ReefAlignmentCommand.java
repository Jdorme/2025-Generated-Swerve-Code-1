package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ReefAlignmentConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class ReefAlignmentCommand extends Command {
    // Subsystem references
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    
    // Alignment configuration constants
    private static final double TARGET_DISTANCE = ReefAlignmentConstants.TARGET_DISTANCE;
    private static final double ALIGNMENT_TOLERANCE = ReefAlignmentConstants.ALIGNMENT_TOLERANCE_METERS;
    private static final double ROTATION_TOLERANCE = ReefAlignmentConstants.ROTATION_TOLERANCE_DEGREES;
    private static final int MAX_DETECTION_ATTEMPTS = ReefAlignmentConstants.MAX_DETECTION_ATTEMPTS;
    private static final double DETECTION_RETRY_DELAY = ReefAlignmentConstants.DETECTION_RETRY_DELAY_SECONDS;
    private static final double ALIGNMENT_TIMEOUT = ReefAlignmentConstants.ALIGNMENT_TIMEOUT_SECONDS;
    private static final int[] VALID_REEF_TAG_IDS = ReefAlignmentConstants.VALID_REEF_TAG_IDS;
    
    // PID Controllers
    private final PIDController xController = new PIDController(
        ReefAlignmentConstants.XAxisPID.kP, 
        ReefAlignmentConstants.XAxisPID.kI, 
        ReefAlignmentConstants.XAxisPID.kD
    );
    private final PIDController yController = new PIDController(
        ReefAlignmentConstants.YAxisPID.kP, 
        ReefAlignmentConstants.YAxisPID.kI, 
        ReefAlignmentConstants.YAxisPID.kD
    );
    private final PIDController rotationController = new PIDController(
        ReefAlignmentConstants.RotationPID.kP, 
        ReefAlignmentConstants.RotationPID.kI, 
        ReefAlignmentConstants.RotationPID.kD
    );
    
    // Swerve requests
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    
    // Side selection enum
    public enum AlignmentSide {
        LEFT, RIGHT
    }
    
    // Command state tracking
    private final AlignmentSide side;
    private int detectionAttempts = 0;
    private double startTime;
    private double lastDetectionAttemptTime;
    private Pose2d bestTagPose;
    
    // Limelight Network Table
    private final NetworkTable reefLimelight;
    
    public ReefAlignmentCommand(
        CommandSwerveDrivetrain drivetrain, 
        VisionSubsystem visionSubsystem,
        AlignmentSide side
    ) {
        // Explicitly require a non-null side
        if (side == null) {
            throw new IllegalArgumentException("Alignment side must be specified (LEFT or RIGHT)");
        }
        
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.side = side;
        
        // Get Reef-side Limelight Network Table
        reefLimelight = NetworkTableInstance.getDefault().getTable("reef");
        
        // Configure PID Controllers
        configureControllers();
        
        // Require the drivetrain subsystem
        addRequirements(drivetrain);
    }
    
    private void configureControllers() {
        xController.setTolerance(ALIGNMENT_TOLERANCE);
        yController.setTolerance(ALIGNMENT_TOLERANCE);
        rotationController.setTolerance(ROTATION_TOLERANCE);
        rotationController.enableContinuousInput(-180, 180);
    }
    
    @Override
    public void initialize() {
        // Reset tracking variables
        detectionAttempts = 0;
        bestTagPose = null;
        
        // Reset PID controllers
        xController.reset();
        yController.reset();
        rotationController.reset();
        
        // Set start time for timeout
        startTime = Timer.getFPGATimestamp();
        lastDetectionAttemptTime = startTime;
    }
    
    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp();
        
        // Check if it's time to attempt tag detection
        if (bestTagPose == null && 
            (currentTime - lastDetectionAttemptTime >= DETECTION_RETRY_DELAY)) {
            
            // Attempt to find the best tag
            bestTagPose = findBestTagForAlignment();
            
            // Increment attempts and update last attempt time
            detectionAttempts++;
            lastDetectionAttemptTime = currentTime;
            
            // Log detection attempt
            SmartDashboard.putNumber("ReefAlignment/DetectionAttempts", detectionAttempts);
            SmartDashboard.putBoolean("ReefAlignment/TagDetected", bestTagPose != null);
        }
        
        // If we have a tag pose, proceed with alignment
        if (bestTagPose != null) {
            // Get current robot pose
            Pose2d currentPose = drivetrain.getState().Pose;
            
            // Calculate desired offset based on side
            Translation2d sideOffset = calculateSideOffset(bestTagPose);
            
            // Calculate error in X, Y, and rotation
            double xError = currentPose.getX() - (bestTagPose.getX() + sideOffset.getX());
            double yError = currentPose.getY() - (bestTagPose.getY() + sideOffset.getY());
            double rotationError = currentPose.getRotation().getDegrees() - bestTagPose.getRotation().getDegrees();
            
            // Calculate PID corrections
            double xCorrection = xController.calculate(xError, 0);
            double yCorrection = yController.calculate(yError, 0);
            double rotationCorrection = rotationController.calculate(rotationError, 0);
            
            // Apply corrections to drivetrain
            drivetrain.setControl(
                driveRequest
                    .withVelocityX(-xCorrection)
                    .withVelocityY(-yCorrection)
                    .withRotationalRate(-rotationCorrection)
            );
            
            // Log alignment details
            SmartDashboard.putNumber("ReefAlignment/XError", xError);
            SmartDashboard.putNumber("ReefAlignment/YError", yError);
            SmartDashboard.putNumber("ReefAlignment/RotationError", rotationError);
        }
    }
    
    private Pose2d findBestTagForAlignment() {
        try {
            // Check if a target is visible
            double targetVisible = reefLimelight.getEntry("tv").getDouble(0.0);
            if (targetVisible < 1.0) {
                return null; // No target visible
            }
            
            // Get tag ID
            double tagId = reefLimelight.getEntry("tid").getDouble(-1);
            
            // Validate tag ID
            boolean validTag = isValidReefTag(tagId);
            if (!validTag) {
                SmartDashboard.putString("ReefAlignment/TagStatus", "Invalid Tag ID: " + tagId);
                return null;
            }
            
            // Get robot pose from Limelight
            double[] botpose = reefLimelight.getEntry("botpose").getDoubleArray(new double[6]);
            if (botpose.length < 6) {
                SmartDashboard.putString("ReefAlignment/TagStatus", "Invalid Pose Data");
                return null;
            }
            
            // Create Pose2d from Limelight data
            Pose2d visionPose = new Pose2d(
                botpose[0], 
                botpose[1],
                Rotation2d.fromDegrees(botpose[5])
            );
            
            // Log successful tag detection
            SmartDashboard.putString("ReefAlignment/TagStatus", "Valid Tag Detected: " + tagId);
            SmartDashboard.putString("ReefAlignment/DetectedPose", 
                String.format("(%.2f, %.2f) %.2f°", 
                    visionPose.getX(), 
                    visionPose.getY(), 
                    visionPose.getRotation().getDegrees()));
            
            return visionPose;
            
        } catch (Exception e) {
            SmartDashboard.putString("ReefAlignment/TagStatus", "Detection Error: " + e.getMessage());
            return null;
        }
    }
    
    private boolean isValidReefTag(double tagId) {
        for (int validId : VALID_REEF_TAG_IDS) {
            if (tagId == validId) {
                return true;
            }
        }
        return false;
    }
    
    private Translation2d calculateSideOffset(Pose2d tagPose) {
        // Determine offset based on side selection
        double offsetDistance = TARGET_DISTANCE;
        
        switch (side) {
            case LEFT:
                // Offset to the left of the tag
                return new Translation2d(
                    -Math.sin(tagPose.getRotation().getRadians()) * offsetDistance,
                    Math.cos(tagPose.getRotation().getRadians()) * offsetDistance
                );
            case RIGHT:
                // Offset to the right of the tag
                return new Translation2d(
                    Math.sin(tagPose.getRotation().getRadians()) * offsetDistance,
                    -Math.cos(tagPose.getRotation().getRadians()) * offsetDistance
                );
            default:
                return new Translation2d(); // No offset
        }
    }
    
    @Override
    public boolean isFinished() {
        // Conditions to end the command:
        // 1. Successfully aligned to the tag
        boolean atTarget = bestTagPose != null && 
                           xController.atSetpoint() && 
                           yController.atSetpoint() && 
                           rotationController.atSetpoint();
        
        // 2. Maximum detection attempts reached
        boolean maxAttemptsReached = detectionAttempts >= MAX_DETECTION_ATTEMPTS;
        
        // 3. Timeout occurred
        boolean timedOut = (Timer.getFPGATimestamp() - startTime) >= ALIGNMENT_TIMEOUT;
        
        // Command ends if any of these conditions are met
        return atTarget || maxAttemptsReached || timedOut;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.setControl(brakeRequest);
        
        // Log command completion status
        if (interrupted) {
            SmartDashboard.putString("ReefAlignment/Status", "Interrupted");
        } else if (bestTagPose == null) {
            SmartDashboard.putString("ReefAlignment/Status", "Failed: No AprilTag detected");
        } else {
            SmartDashboard.putString("ReefAlignment/Status", "Completed Successfully");
        }
    }
}