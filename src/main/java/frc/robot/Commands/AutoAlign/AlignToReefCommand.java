package frc.robot.Commands.AutoAlign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ReefAlignmentConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ReefAlignmentSubsystem;
import frc.robot.subsystems.ReefAlignmentSubsystem.ReefSide;

import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * Command to align the robot to a reef pole using vision.
 */
public class AlignToReefCommand extends Command {
    private final ReefAlignmentSubsystem m_alignmentSubsystem;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final ReefSide m_reefSide;
    
    // State tracking
    private final Timer m_timeoutTimer = new Timer();
    private final Timer m_visionDelayTimer = new Timer();
    private boolean m_hasValidTarget = false;
    private int m_detectionAttempts = 0;
    
    // Control request for driving to position
    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric();
    
    /**
     * Creates a new AlignToReefCommand.
     * 
     * @param alignmentSubsystem The reef alignment subsystem
     * @param drivetrain The swerve drivetrain
     * @param reefSide Which side of the reef to align to
     */
    public AlignToReefCommand(ReefAlignmentSubsystem alignmentSubsystem, 
                              CommandSwerveDrivetrain drivetrain,
                              ReefSide reefSide) {
        m_alignmentSubsystem = alignmentSubsystem;
        m_drivetrain = drivetrain;
        m_reefSide = reefSide;
        
        addRequirements(alignmentSubsystem, drivetrain);
    }
    
    @Override
    public void initialize() {
        // Reset state
        m_hasValidTarget = false;
        m_detectionAttempts = 0;
        m_alignmentSubsystem.reset();
        m_alignmentSubsystem.setTargetSide(m_reefSide);
        
        // Start timers
        m_timeoutTimer.reset();
        m_timeoutTimer.start();
        m_visionDelayTimer.reset();
        m_visionDelayTimer.start();
        
        SmartDashboard.putString("ReefAlign/Status", "Initializing");
        SmartDashboard.putString("ReefAlign/Side", m_reefSide.toString());
    }
    
    @Override
    public void execute() {
        // If we don't have a valid target yet, try to find one
        if (!m_hasValidTarget) {
            // Only attempt detection if enough time has passed since last attempt
            if (m_visionDelayTimer.hasElapsed(ReefAlignmentConstants.DETECTION_RETRY_DELAY_SECONDS)) {
                m_visionDelayTimer.reset();
                m_visionDelayTimer.start();
                
                m_detectionAttempts++;
                SmartDashboard.putNumber("ReefAlign/DetectionAttempts", m_detectionAttempts);
                
                m_hasValidTarget = m_alignmentSubsystem.findAlignmentTarget();
            }
            return; // Skip drive logic until we have a target
        }
        
        // We have a target, execute alignment
        executeAlignment();
    }
    
    private void executeAlignment() {
        // Get current pose
        Pose2d currentPose = m_drivetrain.getState().Pose;
        
        // Calculate drive outputs
        double[] driveOutputs = m_alignmentSubsystem.calculateDriveOutputs(currentPose);
        double xSpeed = driveOutputs[0];
        double ySpeed = driveOutputs[1];
        double rotationSpeed = driveOutputs[2];
        
        // Drive the robot
        m_drivetrain.setControl(
            m_driveRequest
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotationSpeed)
        );
        
        // Debug outputs
        SmartDashboard.putNumber("ReefAlign/XSpeed", xSpeed);
        SmartDashboard.putNumber("ReefAlign/YSpeed", ySpeed);
        SmartDashboard.putNumber("ReefAlign/RotSpeed", rotationSpeed);
        
        // Update status
        if (m_alignmentSubsystem.isAtTarget(currentPose)) {
            SmartDashboard.putString("ReefAlign/Status", "At target");
        } else {
            SmartDashboard.putString("ReefAlign/Status", "Aligning");
        }
    }
    
    @Override
    public boolean isFinished() {
        // Finish if:
        // 1. We've reached the target position within tolerance
        // 2. We've exceeded the timeout
        // 3. We've made too many unsuccessful detection attempts
        
        if (m_hasValidTarget) {
            Pose2d currentPose = m_drivetrain.getState().Pose;
            
            if (m_alignmentSubsystem.isAtTarget(currentPose)) {
                SmartDashboard.putString("ReefAlign/EndReason", "Target reached");
                return true;
            }
        }
        
        if (m_timeoutTimer.hasElapsed(ReefAlignmentConstants.ALIGNMENT_TIMEOUT_SECONDS)) {
            SmartDashboard.putString("ReefAlign/EndReason", "Timeout");
            return true;
        }
        
        if (m_detectionAttempts >= ReefAlignmentConstants.MAX_DETECTION_ATTEMPTS && !m_hasValidTarget) {
            SmartDashboard.putString("ReefAlign/EndReason", "Max detection attempts");
            return true;
        }
        
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        m_drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        
        // Log completion
        if (interrupted) {
            SmartDashboard.putString("ReefAlign/Status", "Interrupted");
        } else {
            SmartDashboard.putString("ReefAlign/Status", "Completed");
        }
    }
}