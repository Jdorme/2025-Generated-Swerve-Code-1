package frc.robot.Commands.AutoAlign;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVisionSubsystem;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class ReefAlignmentCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final PhotonVisionSubsystem m_photonVision;
    private final AlignmentSide m_alignmentSide;
    private final SwerveRequest.RobotCentric m_robotCentricRequest;
    private final SwerveRequest.FieldCentric m_fieldCentricDrive;
    
    // State machine for alignment phases
    private enum AlignmentState {
        ALIGNING,       // Initial alignment to target position
        APPROACHING,    // Moving forward to make contact with the reef
        COMPLETE        // Alignment complete
    }
    
    private AlignmentState m_currentState = AlignmentState.ALIGNING;
    private int m_stableCount = 0;
    private static final int REQUIRED_STABLE_CYCLES = 10; // Require 10 cycles of stability before proceeding

    // Specific tag IDs for different sides
    private static final int LEFT_SIDE_TAG_ID = 8;    // Elevator Cam
    private static final int RIGHT_SIDE_TAG_ID = 8;   // Also Elevator Cam

    // PID controllers for alignment
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_rotationController;
    
    // Slew rate limiters to prevent jerky movements
    private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(2.0);  // 2.0 units/sec^2
    private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(2.0);  // 2.0 units/sec^2
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3.0); // 3.0 rad/sec^2

    // Maximum velocities
    private static final double kMaxTranslationSpeed = 1.5; // meters per second
    private static final double kMaxRotationSpeed = Math.PI; // radians per second
    private static final double kApproachSpeed = 0.5; // meters per second for approaching the reef

    // Alignment thresholds
    private static final double kTranslationTolerance = 0.05;  // 5 cm
    private static final double kRotationTolerance = Math.toRadians(2); // 2 degrees

    // Alignment offsets (fine-tuning of pose)
    private static final Pose2d LEFT_SIDE_POSE_ADJUST = new Pose2d(
        0.35,   // X adjustment (meters)
        -.02,   // Y adjustment (meters)
        Rotation2d.fromDegrees(-90)  // Rotation adjustment
    );

    private static final Pose2d RIGHT_SIDE_POSE_ADJUST = new Pose2d(
        0.35,   // X adjustment (meters)
        -.02,   // Y adjustment (meters)
        Rotation2d.fromDegrees(-90)  // Rotation adjustment
    );
    
    // Set alignment distance to 1 foot (~0.3 meters)
    private static final double kAlignmentDistance = 0.3; // meters in front of the tag

    public enum AlignmentSide {
        LEFT,   // Align to the left side of the Reef AprilTag using Elevator Cam
        RIGHT   // Align to the right side of the Reef AprilTag using Elevator Cam
    }

    public ReefAlignmentCommand(
        CommandSwerveDrivetrain drivetrain, 
        PhotonVisionSubsystem photonVision, 
        AlignmentSide alignmentSide,
        SwerveRequest.FieldCentric fieldCentricDrive
    ) {
        this.m_drivetrain = drivetrain;
        this.m_photonVision = photonVision;
        this.m_alignmentSide = alignmentSide;
        this.m_fieldCentricDrive = fieldCentricDrive;
        
        // Create a robot-centric request for movement
        this.m_robotCentricRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // Set up PID controllers with better tuning
        m_xController = new PIDController(1.75, 0.05, 0.5);
        m_yController = new PIDController(1.75, 0.05, 0.5);
        m_rotationController = new PIDController(2.5, 0.1, 0.75);
        
        // Set tolerances
        m_xController.setTolerance(kTranslationTolerance);
        m_yController.setTolerance(kTranslationTolerance);
        m_rotationController.setTolerance(kRotationTolerance);
        
        // Prevent rotation controller from wrapping around
        m_rotationController.enableContinuousInput(-Math.PI, Math.PI);

        // Require the drivetrain subsystem
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Reset state
        m_currentState = AlignmentState.ALIGNING;
        m_stableCount = 0;
        
        // Reset PID controllers
        m_xController.reset();
        m_yController.reset();
        m_rotationController.reset();
        
        // Reset rate limiters
        m_xLimiter.reset(0);
        m_yLimiter.reset(0);
        m_rotLimiter.reset(0);
        
        // Set preferred tag based on alignment side
        int preferredTagId = (m_alignmentSide == AlignmentSide.LEFT) 
            ? LEFT_SIDE_TAG_ID 
            : RIGHT_SIDE_TAG_ID;
        m_photonVision.setPreferredAprilTag(preferredTagId);
    }

    @Override
    public void execute() {
        // Get current robot pose
        Pose2d currentPose = m_drivetrain.getState().Pose;
        
        switch (m_currentState) {
            case ALIGNING:
                alignToTarget(currentPose);
                break;
                
            case APPROACHING:
                approachReef();
                break;
                
            case COMPLETE:
                // Do nothing, waiting for isFinished to return true
                break;
        }
    }

    private void alignToTarget(Pose2d currentPose) {
        Optional<EstimatedRobotPose> visionPose = m_photonVision.getBestEstimatedPose();
        
        if (visionPose.isPresent()) {
            // Get the tag pose
            Optional<Pose2d> tagPose = m_photonVision.getMeasuredTagPose(
                (m_alignmentSide == AlignmentSide.LEFT) 
                    ? LEFT_SIDE_TAG_ID 
                    : RIGHT_SIDE_TAG_ID, 
                visionPose.get()
            );

            if (tagPose.isPresent()) {
                // Calculate desired alignment pose
                Pose2d desiredPose = calculateDesiredAlignmentPose(tagPose.get());
                
                // Calculate error in robot-relative coordinates
                Pose2d errorPose = desiredPose.relativeTo(currentPose);

                // Use PID controllers to calculate commands
                double xCommand = m_xController.calculate(0, errorPose.getX());
                double yCommand = m_yController.calculate(0, errorPose.getY());
                double rotCommand = m_rotationController.calculate(0, errorPose.getRotation().getRadians());
                
                // Apply rate limiters to smooth motion
                double xVelocity = m_xLimiter.calculate(limitVelocity(xCommand, kMaxTranslationSpeed));
                double yVelocity = m_yLimiter.calculate(limitVelocity(yCommand, kMaxTranslationSpeed));
                double rotVelocity = m_rotLimiter.calculate(limitVelocity(rotCommand, kMaxRotationSpeed));

                // Apply robot-centric request
                m_drivetrain.setControl(
                    m_robotCentricRequest
                        .withVelocityX(xVelocity)
                        .withVelocityY(yVelocity)
                        .withRotationalRate(rotVelocity)
                );
                
                // Check if we're at the target position
                boolean isTranslationOnTarget = 
                    Math.abs(errorPose.getX()) < kTranslationTolerance &&
                    Math.abs(errorPose.getY()) < kTranslationTolerance;

                boolean isRotationOnTarget = 
                    Math.abs(errorPose.getRotation().getRadians()) < kRotationTolerance;
                    
                if (isTranslationOnTarget && isRotationOnTarget) {
                    m_stableCount++;
                    if (m_stableCount >= REQUIRED_STABLE_CYCLES) {
                        // We've been stable for enough cycles, move to approaching phase
                        m_currentState = AlignmentState.APPROACHING;
                    }
                } else {
                    // Reset stability counter if we're not on target
                    m_stableCount = 0;
                }
            }
        }
    }
    
    private void approachReef() {
        // Drive straight toward the reef with controlled speed (only X motion, no Y or rotation)
        m_drivetrain.setControl(
            m_robotCentricRequest
                .withVelocityX(kApproachSpeed)  // Forward speed
                .withVelocityY(0)               // No sideways motion
                .withRotationalRate(0)          // No rotation
        );
        
        // After a fixed time, consider the approach complete
        // This is a simplistic approach - you might want to use a sensor or current draw for better detection
        // of when contact with the reef is made
        if (m_stableCount > REQUIRED_STABLE_CYCLES + 50) { // ~1 second after alignment (assuming 50Hz loop)
            m_currentState = AlignmentState.COMPLETE;
        }
        
        m_stableCount++;
    }

    private Pose2d calculateDesiredAlignmentPose(Pose2d tagPose) {
        // Select the appropriate pose adjustment based on alignment side
        Pose2d sideAdjustment = (m_alignmentSide == AlignmentSide.LEFT) 
            ? LEFT_SIDE_POSE_ADJUST 
            : RIGHT_SIDE_POSE_ADJUST;

        // Calculate desired pose
        return new Pose2d(
            tagPose.getX() - kAlignmentDistance * Math.cos(tagPose.getRotation().getRadians()) + sideAdjustment.getX(),
            tagPose.getY() - kAlignmentDistance * Math.sin(tagPose.getRotation().getRadians()) + sideAdjustment.getY(),
            tagPose.getRotation().plus(sideAdjustment.getRotation())
        );
    }

    private double limitVelocity(double velocity, double maxVelocity) {
        return Math.signum(velocity) * Math.min(Math.abs(velocity), maxVelocity);
    }

    @Override
    public boolean isFinished() {
        return m_currentState == AlignmentState.COMPLETE;
    }

    @Override
    public void end(boolean interrupted) {
        // Return to field-centric drive mode with zero velocity
        m_drivetrain.setControl(
            m_fieldCentricDrive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }
    
    /**
     * Factory method to create a sequential command group that includes 
     * this alignment command followed by a brief wait.
     */
    public static Command createSequential(
        CommandSwerveDrivetrain drivetrain,
        PhotonVisionSubsystem photonVision,
        AlignmentSide side,
        SwerveRequest.FieldCentric fieldCentricDrive
    ) {
        return new SequentialCommandGroup(
            new ReefAlignmentCommand(drivetrain, photonVision, side, fieldCentricDrive),
            new WaitCommand(0.5) // Hold position briefly after completion
        );
    }
}