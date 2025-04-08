package frc.robot.Commands.AutoAlign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
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

    // Specific tag IDs for different sides
    private static final int LEFT_SIDE_TAG_ID = 8;    // Elevator Cam
    private static final int RIGHT_SIDE_TAG_ID = 8;  // Also Elevator Cam

    // PID constants for alignment
    private static final double kP_Translation = 1.75; // Proportional gain for X/Y translation
    private static final double kP_Rotation = 2.5;    // Proportional gain for rotation
    private static final double kMaxTranslationSpeed = 1.5; // meters per second
    private static final double kMaxRotationSpeed = Math.PI; // radians per second

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

        // Require the drivetrain subsystem
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Set preferred tag based on alignment side
        int preferredTagId = (m_alignmentSide == AlignmentSide.LEFT) 
            ? LEFT_SIDE_TAG_ID 
            : RIGHT_SIDE_TAG_ID;
        m_photonVision.setPreferredAprilTag(preferredTagId);
    }

    @Override
    public void execute() {
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
                
                // Calculate current robot pose
                Pose2d currentPose = m_drivetrain.getState().Pose;

                // Calculate translation and rotation errors
                // Transform errors to robot-relative coordinates
                Pose2d errorPose = desiredPose.relativeTo(currentPose);

                // Limit error to max translation/rotation speeds
                double xVelocity = limitVelocity(errorPose.getX() * kP_Translation, kMaxTranslationSpeed);
                double yVelocity = limitVelocity(errorPose.getY() * kP_Translation, kMaxTranslationSpeed);
                double rotationVelocity = limitVelocity(errorPose.getRotation().getRadians() * kP_Rotation, kMaxRotationSpeed);

                // Apply robot-centric request
                m_drivetrain.setControl(
                    m_robotCentricRequest
                        .withVelocityX(xVelocity)
                        .withVelocityY(yVelocity)
                        .withRotationalRate(rotationVelocity)
                );
            }
        }
    }

    private Pose2d calculateDesiredAlignmentPose(Pose2d tagPose) {
        // Select the appropriate pose adjustment based on alignment side
        Pose2d sideAdjustment = (m_alignmentSide == AlignmentSide.LEFT) 
            ? LEFT_SIDE_POSE_ADJUST 
            : RIGHT_SIDE_POSE_ADJUST;

        // Distance from tag to align (adjust based on your game's requirements)
        double alignmentDistance = .22; // meters in front of the tag

        // Calculate desired pose
        return new Pose2d(
            tagPose.getX() - alignmentDistance * Math.cos(tagPose.getRotation().getRadians()) + sideAdjustment.getX(),
            tagPose.getY() - alignmentDistance * Math.sin(tagPose.getRotation().getRadians()) + sideAdjustment.getY(),
            tagPose.getRotation().plus(sideAdjustment.getRotation())
        );
    }

    private double limitVelocity(double velocity, double maxVelocity) {
        return Math.signum(velocity) * Math.min(Math.abs(velocity), maxVelocity);
    }

    @Override
    public boolean isFinished() {
        // Get current robot pose
        Pose2d currentPose = m_drivetrain.getState().Pose;
        Optional<EstimatedRobotPose> visionPose = m_photonVision.getBestEstimatedPose();

        if (visionPose.isPresent()) {
            Optional<Pose2d> tagPose = m_photonVision.getMeasuredTagPose(
                (m_alignmentSide == AlignmentSide.LEFT) 
                    ? LEFT_SIDE_TAG_ID 
                    : RIGHT_SIDE_TAG_ID, 
                visionPose.get()
            );

            if (tagPose.isPresent()) {
                Pose2d desiredPose = calculateDesiredAlignmentPose(tagPose.get());

                // Calculate error in robot-relative coordinates
                Pose2d errorPose = desiredPose.relativeTo(currentPose);

                // Check if within translation and rotation tolerances
                boolean isTranslationOnTarget = 
                    Math.abs(errorPose.getX()) < kTranslationTolerance &&
                    Math.abs(errorPose.getY()) < kTranslationTolerance;

                boolean isRotationOnTarget = 
                    Math.abs(errorPose.getRotation().getRadians()) < kRotationTolerance;

                return isTranslationOnTarget && isRotationOnTarget;
            }
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Return to field-centric drive mode
        m_drivetrain.setControl(
            m_fieldCentricDrive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }
}