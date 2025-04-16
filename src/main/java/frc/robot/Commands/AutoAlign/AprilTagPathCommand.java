package frc.robot.Commands.AutoAlign;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVisionSubsystem;

/**
 * Command to generate and follow a path to the nearest allowed AprilTag
 * using camera-based measurements for tag positions
 */
public class AprilTagPathCommand extends Command {
    
    private final PhotonVisionSubsystem m_photonVision;
    private final CommandSwerveDrivetrain m_drivetrain;
    
    private Command m_pathCommand = null;
    private boolean m_isPoseValid = false;
    private boolean m_isPathGenerated = false;
    
    // List of allowed tag IDs
    private final Set<Integer> m_allowedTagIds;
    Object m_currentAlliance;
    // Path planning constants
    private final double MAX_VELOCITY; // m/s
    private final double MAX_ACCELERATION; // m/s²
    private final double Y_OFFSET; // meters (offset along tag's Y-axis)
    
    /**
     * Create a command to generate and follow a path to the nearest allowed AprilTag
     * @param photonVision The PhotonVision subsystem
     * @param drivetrain The swerve drivetrain
     * @param yOffset The offset along the tag's local Y-axis (positive = left of tag)
     * @param maxVelocity Maximum velocity for path following (m/s)
     * @param maxAcceleration Maximum acceleration for path following (m/s²)
     * @param allowedTagIds Array of allowed tag IDs to target
     */
    public AprilTagPathCommand(
            PhotonVisionSubsystem photonVision, 
            CommandSwerveDrivetrain drivetrain,
            double yOffset,
            double maxVelocity,
            double maxAcceleration,
            Integer... allowedTagIds) {
        
        m_photonVision = photonVision;
        m_drivetrain = drivetrain;
        Y_OFFSET = yOffset;
        MAX_VELOCITY = maxVelocity;
        MAX_ACCELERATION = maxAcceleration;
        
        // Convert array to set for efficient lookups
        m_allowedTagIds = new HashSet<>(Arrays.asList(allowedTagIds));
        
        addRequirements(drivetrain);
        
        // Display status on SmartDashboard
        SmartDashboard.putBoolean("TagPath/PoseValid", false);
        SmartDashboard.putBoolean("TagPath/PathGenerated", false);
        SmartDashboard.putNumber("TagPath/TargetTagId", -1);
        SmartDashboard.putBoolean("TagPath/ValidTagFound", false);
    }
    
    @Override
    public void initialize() {
        // Reset state
        m_pathCommand = null;
        m_isPoseValid = false;
        m_isPathGenerated = false;
        
        // Reset display values
        SmartDashboard.putBoolean("TagPath/PoseValid", false);
        SmartDashboard.putBoolean("TagPath/PathGenerated", false);
        SmartDashboard.putNumber("TagPath/TargetTagId", -1);
        SmartDashboard.putBoolean("TagPath/ValidTagFound", false);
        // Get the current alliance
    Optional<Alliance> alliance = DriverStation.getAlliance();
    m_currentAlliance = alliance.isPresent() ? alliance.get() : Alliance.Blue;
        DriverStation.reportWarning("AprilTagPath initialized. Looking for nearest allowed tag.", false);
    }
    
    @Override
    public void execute() {
        // First, make sure we have a valid robot pose from vision
        if (!m_isPoseValid) {
            Optional<EstimatedRobotPose> robotPoseOptional = m_photonVision.getBestEstimatedPose();
            
            // Check if we have a valid pose
            if (robotPoseOptional.isPresent()) {
                EstimatedRobotPose pose = robotPoseOptional.get();
                
                // Update pose confidence to determine validity
                double confidence = m_photonVision.calculatePoseConfidence(pose);
                if (confidence > 0.3) { // Minimum confidence threshold
                    m_isPoseValid = true;
                    SmartDashboard.putBoolean("TagPath/PoseValid", true);
                    DriverStation.reportWarning("Valid pose found", false);
                    
                    // Once we have a valid pose, proceed with path generation using that same pose
                    generatePathWithValidPose(pose);
                }
            }
        }
        
        // Once the path is generated, schedule it
        if (m_isPathGenerated && m_pathCommand != null) {
            m_pathCommand.schedule();
            m_pathCommand = null; // Don't schedule it again
        }
    }
    
    /**
     * Generate a path using a validated pose
     * This ensures we use the same vision result for all calculations
     * @param pose The validated EstimatedRobotPose from vision
     */
    private void generatePathWithValidPose(EstimatedRobotPose pose) {
        try {
            // Get the nearest visible tag ID using the provided pose
            int nearestTagId = m_photonVision.getNearestVisibleTagId(pose);
            
            // Check if the nearest tag is in our allowed list
            if (nearestTagId != -1 && m_allowedTagIds.contains(nearestTagId)) {
                SmartDashboard.putBoolean("TagPath/ValidTagFound", true);
                SmartDashboard.putNumber("TagPath/TargetTagId", nearestTagId);
                
                // Get the field position of the nearest tag using camera measurements
                // and the same pose result
                Optional<Pose2d> tagPoseOptional = m_photonVision.getMeasuredTagPose(nearestTagId, pose);
                
                if (tagPoseOptional.isPresent()) {
                    Pose2d tagPose = tagPoseOptional.get();
                    
                    // Calculate approach position with Y offset
                    Pose2d currentPose = m_drivetrain.getState().Pose;
                    Pose2d goalPose = calculateApproachPose(tagPose, currentPose, Y_OFFSET);
                    
                    // Generate the path
                    m_pathCommand = generatePathCommand(currentPose, goalPose);
                    m_isPathGenerated = true;
                    SmartDashboard.putBoolean("TagPath/PathGenerated", true);
                    DriverStation.reportWarning("Path generated to nearest allowed tag ID: " + nearestTagId, false);
                } else {
                    DriverStation.reportWarning("Could not determine tag position for tag ID: " + nearestTagId, false);
                }
            } else {
                // Either no tag was found or nearest tag isn't in the allowed list
                if (nearestTagId == -1) {
                    DriverStation.reportWarning("No visible tags found", false);
                } else {
                    DriverStation.reportWarning("Nearest tag ID " + nearestTagId + " is not in allowed list", false);
                }
                SmartDashboard.putBoolean("TagPath/ValidTagFound", false);
            }
        } catch (Exception e) {
            DriverStation.reportError("Error generating path: " + e.getMessage(), e.getStackTrace());
        }
    }
    
    /**
     * Calculate a goal pose beside the tag with Y-axis offset
     * @param tagPose The pose of the AprilTag
     * @param currentPose The current robot pose
     * @param yOffset The offset along the tag's local Y-axis (positive = to the left of the tag)
     * @return The goal pose beside the tag
     */
    private Pose2d calculateApproachPose(Pose2d tagPose, Pose2d currentPose, double yOffset) {
      // Get the tag's rotation
      Rotation2d tagRotation = tagPose.getRotation();
      
      double originalYOffset = yOffset;
      
      // For Red alliance, we need to adjust the Y offset direction
      if (m_currentAlliance == Alliance.Red) {
          // Flip the Y offset for Red alliance
          yOffset = -yOffset;
          DriverStation.reportWarning("Red alliance detected, flipping Y offset from " + 
                                    originalYOffset + " to " + yOffset, false);
      } else {
          DriverStation.reportWarning("Blue alliance detected, keeping Y offset as " + yOffset, false);
      }
      
      // Calculate the vector that represents the tag's Y-axis in field coordinates
      // This is perpendicular to the tag's heading
      Translation2d tagYAxis = new Translation2d(
          Math.cos(tagRotation.getRadians() + Math.PI/2),
          Math.sin(tagRotation.getRadians() + Math.PI/2)
      );
      
      // Calculate the offset position along the tag's Y-axis
      Translation2d offsetPosition = tagPose.getTranslation()
          .plus(tagYAxis.times(1));
      
      // Calculate the goal rotation (90 degrees from tag heading)
      // Adjust based on alliance
      Rotation2d goalRotation;
      if (m_currentAlliance == Alliance.Red) {
          goalRotation = tagRotation.plus(Rotation2d.fromDegrees(-90));
      } else {
          goalRotation = tagRotation.plus(Rotation2d.fromDegrees(90));
      }
      
      DriverStation.reportWarning("Tag at (" + tagPose.getX() + "," + tagPose.getY() + 
                               "), Calculated goal at (" + offsetPosition.getX() + "," + 
                               offsetPosition.getY() + ")", false);
      
      // Return the goal pose
      return new Pose2d(offsetPosition, goalRotation);
  }
    
    /**
     * Generate a path command from current pose to goal pose
     * @param startPose The starting pose
     * @param endPose The ending pose
     * @return A command that follows the generated path
     */
    private Command generatePathCommand(Pose2d startPose, Pose2d endPose) {
      // Create path constraints
      PathConstraints constraints = new PathConstraints(
          MAX_VELOCITY, MAX_ACCELERATION,
          Math.toRadians(540), Math.toRadians(720) // Default rotation constraints
      );
      
      // Build a path
      PathPlannerPath path = new PathPlannerPath(
          PathPlannerPath.waypointsFromPoses(
              startPose,
              endPose
          ),
          constraints,
          null, // IdealStartingState parameter (can be null for on-the-fly paths)
          new GoalEndState(0, endPose.getRotation())
      );
      
      // Log that we're generating a path for the current alliance
      DriverStation.reportWarning("Generated path for " + m_currentAlliance + 
                                 " alliance from (" + startPose.getX() + "," + startPose.getY() + ") to (" + 
                                 endPose.getX() + "," + endPose.getY() + ")", false);
      
      // This approach tells AutoBuilder to follow the path
      // We'll respect the current alliance detection (don't force flip)
      // But we'll disable automatic flipping that might be happening elsewhere
      return AutoBuilder.followPath(path.flipPath());
  }
    
    @Override
    public boolean isFinished() {
        // The command is finished when the path is generated and scheduled
        // OR if we've determined no valid tag was found
        return (m_isPathGenerated && m_pathCommand == null) || 
               (m_isPoseValid && SmartDashboard.getBoolean("TagPath/ValidTagFound", false) == false);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Update SmartDashboard
        SmartDashboard.putBoolean("TagPath/PoseValid", false);
        SmartDashboard.putBoolean("TagPath/PathGenerated", false);
        
        if (interrupted) {
            DriverStation.reportWarning("AprilTagPath command was interrupted", false);
        } else {
            DriverStation.reportWarning("AprilTagPath command completed successfully", false);
        }
    }
}