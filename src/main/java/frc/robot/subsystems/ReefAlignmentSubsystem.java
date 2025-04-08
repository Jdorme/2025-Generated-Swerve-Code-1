package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ReefAlignmentConstants;

/**
 * Subsystem that handles alignment to the reef poles using vision data.
 */
public class ReefAlignmentSubsystem extends SubsystemBase {
    public enum ReefSide {
        LEFT_POLE,
        RIGHT_POLE
    }
    
    // Valid AprilTag IDs for reef alignment - retrieved from Constants
    private final int[] BLUE_ALLIANCE_REEF_TAGS = ReefAlignmentConstants.VALID_REEF_TAG_IDS_BLUE;
    private final int[] RED_ALLIANCE_REEF_TAGS = ReefAlignmentConstants.VALID_REEF_TAG_IDS_RED;
    
    // Subsystem dependencies
    private final PhotonVisionSubsystem m_photonVision;
    
    // PID controllers for alignment
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_rotationController;
    
    // Alignment state
    private ReefSide m_targetSide = ReefSide.LEFT_POLE;
    private Translation2d m_leftPoleOffset = new Translation2d(0.6, 0.3);  // X forward, Y left
    private Translation2d m_rightPoleOffset = new Translation2d(0.6, -0.3); // X forward, Y left
    private Pose2d m_targetPose = null;
    private Pose2d m_lastTagPose = null;
    private int m_lastTagId = -1;
    
    /**
     * Creates a new ReefAlignmentSubsystem.
     * 
     * @param photonVision The PhotonVision subsystem
     */
    public ReefAlignmentSubsystem(PhotonVisionSubsystem photonVision) {
        m_photonVision = photonVision;
        
        // Configure PID controllers with initial values
        m_xController = new PIDController(
            ReefAlignmentConstants.XAxisPID.kP,
            ReefAlignmentConstants.XAxisPID.kI,
            ReefAlignmentConstants.XAxisPID.kD);
        
        m_yController = new PIDController(
            ReefAlignmentConstants.YAxisPID.kP,
            ReefAlignmentConstants.YAxisPID.kI,
            ReefAlignmentConstants.YAxisPID.kD);
        
        m_rotationController = new PIDController(
            ReefAlignmentConstants.RotationPID.kP,
            ReefAlignmentConstants.RotationPID.kI,
            ReefAlignmentConstants.RotationPID.kD);
        
        m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        // Publish initial tuning values
        publishTuningParameters();
    }
    
    /**
     * Updates the side of the reef to target.
     * 
     * @param side The side of the reef to target
     */
    public void setTargetSide(ReefSide side) {
        m_targetSide = side;
        SmartDashboard.putString("ReefAlign/TargetSide", side.toString());
    }
    
    /**
     * Updates the offset values for alignment.
     * 
     * @param leftPoleX X offset for left pole
     * @param leftPoleY Y offset for left pole
     * @param rightPoleX X offset for right pole
     * @param rightPoleY Y offset for right pole
     */
    public void setAlignmentOffsets(double leftPoleX, double leftPoleY, double rightPoleX, double rightPoleY) {
        m_leftPoleOffset = new Translation2d(leftPoleX, leftPoleY);
        m_rightPoleOffset = new Translation2d(rightPoleX, rightPoleY);
        
        SmartDashboard.putNumber("ReefAlign/LeftOffsetX", leftPoleX);
        SmartDashboard.putNumber("ReefAlign/LeftOffsetY", leftPoleY);
        SmartDashboard.putNumber("ReefAlign/RightOffsetX", rightPoleX);
        SmartDashboard.putNumber("ReefAlign/RightOffsetY", rightPoleY);
    }
    
    /**
     * Updates the PID values for alignment.
     * 
     * @param xP P value for X axis control
     * @param xI I value for X axis control
     * @param xD D value for X axis control
     * @param yP P value for Y axis control
     * @param yI I value for Y axis control
     * @param yD D value for Y axis control
     * @param rotP P value for rotation control
     * @param rotI I value for rotation control
     * @param rotD D value for rotation control
     */
    public void updatePIDValues(double xP, double xI, double xD, 
                               double yP, double yI, double yD, 
                               double rotP, double rotI, double rotD) {
        m_xController.setPID(xP, xI, xD);
        m_yController.setPID(yP, yI, yD);
        m_rotationController.setPID(rotP, rotI, rotD);
    }
    
    /**
     * Tries to find a valid alignment target using vision.
     * 
     * @return True if a valid target was found
     */
    public boolean findAlignmentTarget() {
        // Get the latest vision measurement
        Optional<EstimatedRobotPose> visionPose = m_photonVision.getBestEstimatedPose();
        
        if (!visionPose.isPresent() || visionPose.get().targetsUsed.isEmpty()) {
            SmartDashboard.putString("ReefAlign/Status", "No vision data available");
            return false;
        }
        
        EstimatedRobotPose pose = visionPose.get();
        
        // Check confidence
        double confidence = m_photonVision.calculatePoseConfidence(pose);
        SmartDashboard.putNumber("ReefAlign/VisionConfidence", confidence);
        
        if (confidence < 0.5) { // Minimum confidence threshold
            SmartDashboard.putString("ReefAlign/Status", "Low confidence vision data");
            return false;
        }
        
        // Get the nearest visible tag ID
        int nearestTagId = m_photonVision.getNearestVisibleTagId(pose);
        SmartDashboard.putNumber("ReefAlign/NearestTagId", nearestTagId);
        m_lastTagId = nearestTagId;
        
        // Check if this is a valid reef tag for our alliance
        boolean isRedAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        int[] validTags = isRedAlliance ? RED_ALLIANCE_REEF_TAGS : BLUE_ALLIANCE_REEF_TAGS;
        
        boolean isValidReefTag = Arrays.stream(validTags).anyMatch(id -> id == nearestTagId);
        
        if (!isValidReefTag) {
            SmartDashboard.putString("ReefAlign/Status", "Not a valid reef tag: " + nearestTagId);
            return false;
        }
        
        // Calculate the target pose with the appropriate offset
        try {
            // Get tag pose from PhotonVision
            Optional<Pose2d> tagPoseMeasured = m_photonVision.getMeasuredTagPose(nearestTagId, pose);
            
            if (!tagPoseMeasured.isPresent()) {
                SmartDashboard.putString("ReefAlign/Status", "Failed to get tag pose");
                return false;
            }
            
            Pose2d tagPose = tagPoseMeasured.get();
            m_lastTagPose = tagPose;
            
            SmartDashboard.putString("ReefAlign/TagPose", 
                String.format("(%.2f, %.2f, %.1f°)", 
                tagPose.getX(), tagPose.getY(), tagPose.getRotation().getDegrees()));
            
            // Get the appropriate offset for the selected side
            Translation2d offset = (m_targetSide == ReefSide.LEFT_POLE) ? 
                m_leftPoleOffset : m_rightPoleOffset;
                
            // Apply rotation to offset based on tag orientation
            Translation2d rotatedOffset = offset;
            
            // For red alliance, we need different offsets
            if (isRedAlliance) {
                // Invert X for red alliance (facing opposite direction)
                rotatedOffset = new Translation2d(-offset.getX(), offset.getY());
            }
            
            // Now rotate the offset based on tag orientation
            rotatedOffset = rotatedOffset.rotateBy(tagPose.getRotation());
            
            // Determine the target rotation - face the tag
            Rotation2d targetRotation = tagPose.getRotation();
            if (isRedAlliance) {
                // For red alliance, we need to face the opposite direction
                targetRotation = targetRotation.plus(Rotation2d.fromDegrees(180));
            }
            
            // Create the target pose
            m_targetPose = new Pose2d(
                tagPose.getX() + rotatedOffset.getX(), 
                tagPose.getY() + rotatedOffset.getY(),
                targetRotation
            );
            
            SmartDashboard.putString("ReefAlign/TargetPose", 
                String.format("(%.2f, %.2f, %.1f°)", 
                m_targetPose.getX(), m_targetPose.getY(), m_targetPose.getRotation().getDegrees()));
            
            SmartDashboard.putString("ReefAlign/Status", "Target acquired");
            return true;
        } catch (Exception e) {
            SmartDashboard.putString("ReefAlign/Status", "Error calculating target: " + e.getMessage());
            return false;
        }
    }
    
    /**
     * Calculates the drive outputs to reach the target position.
     * 
     * @param currentPose The current robot pose
     * @return Array of drive outputs [xSpeed, ySpeed, rotationSpeed]
     */
    public double[] calculateDriveOutputs(Pose2d currentPose) {
        if (m_targetPose == null) {
            return new double[] {0, 0, 0};
        }
        
        // Calculate distance to target
        double xDistance = m_targetPose.getX() - currentPose.getX();
        double yDistance = m_targetPose.getY() - currentPose.getY();
        double distance = Math.sqrt(xDistance * xDistance + yDistance * yDistance);
        
        // Calculate rotation difference
        double rotationDifference = currentPose.getRotation().minus(m_targetPose.getRotation()).getRadians();
        
        SmartDashboard.putNumber("ReefAlign/Distance", distance);
        SmartDashboard.putNumber("ReefAlign/RotationDiff", rotationDifference * 180 / Math.PI);
        
        // Calculate drive outputs using PID
        double xSpeed = m_xController.calculate(currentPose.getX(), m_targetPose.getX());
        double ySpeed = m_yController.calculate(currentPose.getY(), m_targetPose.getY());
        double rotationSpeed = m_rotationController.calculate(
            currentPose.getRotation().getRadians(), 
            m_targetPose.getRotation().getRadians()
        );
        
        // Apply speed limits
        double maxLinearSpeed = 2.0; // meters per second
        double maxRotationalSpeed = 1.5; // radians per second
        
        xSpeed = clamp(xSpeed, -maxLinearSpeed, maxLinearSpeed);
        ySpeed = clamp(ySpeed, -maxLinearSpeed, maxLinearSpeed);
        rotationSpeed = clamp(rotationSpeed, -maxRotationalSpeed, maxRotationalSpeed);
        
        // If we're close to target, reduce speeds
        if (distance < 0.3) {
            xSpeed *= 0.5;
            ySpeed *= 0.5;
        }
        
        if (Math.abs(rotationDifference) < 0.2) {
            rotationSpeed *= 0.5;
        }
        
        // Return the calculated outputs
        return new double[] {xSpeed, ySpeed, rotationSpeed};
    }
    
    /**
     * Checks if the robot is at the target position within tolerance.
     * 
     * @param currentPose The current robot pose
     * @return True if the robot is at the target position
     */
    public boolean isAtTarget(Pose2d currentPose) {
        if (m_targetPose == null) {
            return false;
        }
        
        double distance = currentPose.getTranslation().getDistance(m_targetPose.getTranslation());
        double rotationDifference = Math.abs(currentPose.getRotation().minus(m_targetPose.getRotation()).getRadians());
        
        return distance < ReefAlignmentConstants.ALIGNMENT_TOLERANCE_METERS && 
               rotationDifference < Math.toRadians(ReefAlignmentConstants.ROTATION_TOLERANCE_DEGREES);
    }
    
    /**
     * Gets the current target pose.
     * 
     * @return The target pose, or null if not set
     */
    public Pose2d getTargetPose() {
        return m_targetPose;
    }
    
    /**
     * Gets the most recent tag pose that was detected.
     * 
     * @return The tag pose, or null if none detected
     */
    public Pose2d getLastTagPose() {
        return m_lastTagPose;
    }
    
    /**
     * Gets the most recent tag ID that was detected.
     * 
     * @return The tag ID, or -1 if none detected
     */
    public int getLastTagId() {
        return m_lastTagId;
    }
    
    /**
     * Reset the target pose and other alignment state.
     */
    public void reset() {
        m_targetPose = null;
        m_lastTagPose = null;
        m_lastTagId = -1;
    }
    
    @Override
    public void periodic() {
        // Update PID values from SmartDashboard if tuning is enabled
        if (SmartDashboard.getBoolean("ReefAlign/Tuning/Enabled", false)) {
            updatePIDFromDashboard();
        }
    }
    
    /**
     * Helper method to update PID values from SmartDashboard.
     */
    private void updatePIDFromDashboard() {
        double xP = SmartDashboard.getNumber("ReefAlign/Tuning/X/P", ReefAlignmentConstants.XAxisPID.kP);
        double xI = SmartDashboard.getNumber("ReefAlign/Tuning/X/I", ReefAlignmentConstants.XAxisPID.kI);
        double xD = SmartDashboard.getNumber("ReefAlign/Tuning/X/D", ReefAlignmentConstants.XAxisPID.kD);
        
        double yP = SmartDashboard.getNumber("ReefAlign/Tuning/Y/P", ReefAlignmentConstants.YAxisPID.kP);
        double yI = SmartDashboard.getNumber("ReefAlign/Tuning/Y/I", ReefAlignmentConstants.YAxisPID.kI);
        double yD = SmartDashboard.getNumber("ReefAlign/Tuning/Y/D", ReefAlignmentConstants.YAxisPID.kD);
        
        double rotP = SmartDashboard.getNumber("ReefAlign/Tuning/Rotation/P", ReefAlignmentConstants.RotationPID.kP);
        double rotI = SmartDashboard.getNumber("ReefAlign/Tuning/Rotation/I", ReefAlignmentConstants.RotationPID.kI);
        double rotD = SmartDashboard.getNumber("ReefAlign/Tuning/Rotation/D", ReefAlignmentConstants.RotationPID.kD);
        
        updatePIDValues(xP, xI, xD, yP, yI, yD, rotP, rotI, rotD);
    }
    
    /**
     * Helper method to clamp a value within a range.
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    /**
     * Publish tuning parameters to SmartDashboard.
     */
    private void publishTuningParameters() {
        SmartDashboard.putBoolean("ReefAlign/Tuning/Enabled", false);
        
        // Position tuning
        SmartDashboard.putNumber("ReefAlign/Tuning/LeftOffsetX", m_leftPoleOffset.getX());
        SmartDashboard.putNumber("ReefAlign/Tuning/LeftOffsetY", m_leftPoleOffset.getY());
        SmartDashboard.putNumber("ReefAlign/Tuning/RightOffsetX", m_rightPoleOffset.getX());
        SmartDashboard.putNumber("ReefAlign/Tuning/RightOffsetY", m_rightPoleOffset.getY());
        
        // PID tuning
        SmartDashboard.putNumber("ReefAlign/Tuning/X/P", ReefAlignmentConstants.XAxisPID.kP);
        SmartDashboard.putNumber("ReefAlign/Tuning/X/I", ReefAlignmentConstants.XAxisPID.kI);
        SmartDashboard.putNumber("ReefAlign/Tuning/X/D", ReefAlignmentConstants.XAxisPID.kD);
        
        SmartDashboard.putNumber("ReefAlign/Tuning/Y/P", ReefAlignmentConstants.YAxisPID.kP);
        SmartDashboard.putNumber("ReefAlign/Tuning/Y/I", ReefAlignmentConstants.YAxisPID.kI);
        SmartDashboard.putNumber("ReefAlign/Tuning/Y/D", ReefAlignmentConstants.YAxisPID.kD);
        
        SmartDashboard.putNumber("ReefAlign/Tuning/Rotation/P", ReefAlignmentConstants.RotationPID.kP);
        SmartDashboard.putNumber("ReefAlign/Tuning/Rotation/I", ReefAlignmentConstants.RotationPID.kI);
        SmartDashboard.putNumber("ReefAlign/Tuning/Rotation/D", ReefAlignmentConstants.RotationPID.kD);
    }
}