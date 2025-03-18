package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 * Modified to integrate with PhotonVision for pose estimation with
 * enhanced pose rejection logic.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* PhotonVision integration */
    private PhotonVisionSubsystem m_photonVision;
    private boolean m_useVision = true;
    private double m_lastVisionUpdateTime = 0;
    private static final double kVisionUpdateRateLimit = 0.2; // Limit vision updates to 5 Hz (every 200ms)

    private Pose2d m_lastVisionPose = null;
    private int m_consistentVisionCount = 0;
    
    /* Standard deviations for vision measurements */
    private Matrix<N3, N1> m_defaultVisionStdDevs;
    private Matrix<N3, N1> m_farVisionStdDevs;
    private final double kFarVisionThreshold = 4.0; // meters - distance beyond which vision is considered "far"

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
    
    /**
     * Enhanced pose rejection logic components
     */
    
    /**
     * Class for storing historical pose data for tracking odometry health
     */
    private static class PoseHistoryEntry {
        public Pose2d pose;
        public double timestamp;
        
        public PoseHistoryEntry(Pose2d pose, double timestamp) {
            this.pose = pose;
            this.timestamp = timestamp;
        }
    }

    // Constants for pose rejection logic
    private static final int MAX_POSE_HISTORY_SIZE = 20;
    private static final double MAX_VELOCITY_METERS_PER_SEC = 4.0; // Maximum physical speed of your robot
    private static final double MAX_ROTATION_RATE_DEG_PER_SEC = 540.0; // Maximum rotation rate

    // Variables for tracking pose history
    private final List<PoseHistoryEntry> poseHistory = new ArrayList<>();
    private Pose2d lastAcceptedVisionPose = null;
    private double lastAcceptedVisionTime = 0;

    /**
     * Result class for pose validation
     */
    private static class PoseValidationResult {
        public boolean valid;
        public String reason;
        public double confidenceScale;
        
        public PoseValidationResult(boolean valid, String reason, double confidenceScale) {
            this.valid = valid;
            this.reason = reason;
            this.confidenceScale = confidenceScale;
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants and PhotonVision subsystem.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        m_photonVision = new PhotonVisionSubsystem();
        initializeVisionStdDevs();
        
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants and PhotonVision subsystem.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop
  
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        m_photonVision = new PhotonVisionSubsystem();
        initializeVisionStdDevs();
        
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants and PhotonVision subsystem.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     * @param visionStandardDeviation   The standard deviation for vision calculation

     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        m_photonVision = new PhotonVisionSubsystem();
        m_defaultVisionStdDevs = visionStandardDeviation;
        initializeVisionStdDevs();
        
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Initialize the standard deviations for vision measurements
     */
    private void initializeVisionStdDevs() {
        // If default vision std devs weren't initialized in the constructor
        if (m_defaultVisionStdDevs == null) {
            // Create default standard deviations - moderate trust in vision
            m_defaultVisionStdDevs = new Matrix<>(N3.instance, N1.instance);
            m_defaultVisionStdDevs.set(0, 0, .25);  // X standard deviation (meters)0.25
            m_defaultVisionStdDevs.set(1, 0, .25);  // Y standard deviation (meters).25
            m_defaultVisionStdDevs.set(2, 0, 0.25);  // Rotation standard deviation (radians).1
        }
        
        // Create higher standard deviations for far vision measurements - less trust
        m_farVisionStdDevs = new Matrix<>(N3.instance, N1.instance);
        m_farVisionStdDevs.set(0, 0, 1.5);  // X standard deviation (meters) 1.5
        m_farVisionStdDevs.set(1, 0, 1.5);  // Y standard deviation (meters)
        m_farVisionStdDevs.set(2, 0, 0.25);  // Rotation standard deviation (radians)
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Set the PhotonVision subsystem to use for pose estimation
     * @param photonVision The PhotonVision subsystem to use
     */
    public void setPhotonVision(PhotonVisionSubsystem photonVision) {
        m_photonVision = photonVision;
        SmartDashboard.putBoolean("Drivetrain/HasVision", photonVision != null);
    }
    
    /**
     * Enable or disable the use of vision for pose estimation
     * @param useVision Whether to use vision for pose estimation
     */
    public void setUseVision(boolean useVision) {
        m_useVision = useVision;
        SmartDashboard.putBoolean("Drivetrain/UseVision", useVision);
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        // Apply operator perspective if needed
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        
        // Update pose with vision measurements
        updatePoseWithVision();
        
        // Log current pose to SmartDashboard
        Pose2d currentPose = getState().Pose;
        SmartDashboard.putNumber("Drivetrain/Pose/X", currentPose.getX());
        SmartDashboard.putNumber("Drivetrain/Pose/Y", currentPose.getY());
        SmartDashboard.putNumber("Drivetrain/Pose/Rotation", currentPose.getRotation().getDegrees());
    }
    
    /**
     * Validates a vision pose measurement against historical data and physical constraints
     * 
     * @param visionPose The vision pose to validate
     * @param currentPose The current odometry pose
     * @param timestamp The timestamp of the vision measurement
     * @return A validation result with reason and confidence adjustment
     */
    private PoseValidationResult validateVisionPose(Pose2d visionPose, Pose2d currentPose, double timestamp) {
        // Calculate differences from current pose
        double positionDifference = currentPose.getTranslation().getDistance(visionPose.getTranslation());
        double rotationDifference = Math.abs(currentPose.getRotation().minus(visionPose.getRotation()).getDegrees());
        
        // Log the differences for debugging
        SmartDashboard.putNumber("Drivetrain/Vision/PositionDifference", positionDifference);
        SmartDashboard.putNumber("Drivetrain/Vision/RotationDifference", rotationDifference);
        
        // Check for physically impossible movement (teleportation)
        if (lastAcceptedVisionPose != null && lastAcceptedVisionTime > 0) {
            double timeDelta = timestamp - lastAcceptedVisionTime;
            if (timeDelta > 0) {
                double distanceDelta = lastAcceptedVisionPose.getTranslation()
                    .getDistance(visionPose.getTranslation());
                double rotationDelta = Math.abs(lastAcceptedVisionPose.getRotation()
                    .minus(visionPose.getRotation()).getDegrees());
                    
                double measuredVelocity = distanceDelta / timeDelta;
                double measuredRotationRate = rotationDelta / timeDelta;
                
                // Log measured rates for tuning
                SmartDashboard.putNumber("Drivetrain/Vision/MeasuredVelocity", measuredVelocity);
                SmartDashboard.putNumber("Drivetrain/Vision/MeasuredRotationRate", measuredRotationRate);
                
                // Check if the required velocity would be physically impossible
                double velocityBuffer = 1.25; // 25% buffer over max velocity
                if (measuredVelocity > MAX_VELOCITY_METERS_PER_SEC * velocityBuffer) {
                    return new PoseValidationResult(false, 
                        "Physically impossible movement: " + String.format("%.2f", measuredVelocity) + " m/s", 0.0);
                }
                
                // Check if rotation rate would be physically impossible
                double rotationBuffer = 1.25; // 25% buffer over max rotation rate
                if (measuredRotationRate > MAX_ROTATION_RATE_DEG_PER_SEC * rotationBuffer) {
                    return new PoseValidationResult(false, 
                        "Physically impossible rotation: " + String.format("%.2f", measuredRotationRate) + " deg/s", 0.0);
                }
            }
        }
        
        // Check if robot is stationary but vision poses are jumping around
        boolean isRobotStationary = isRobotStationary();
        if (isRobotStationary && positionDifference > 0.3) {
            // Robot is stationary but vision poses are changing significantly
            return new PoseValidationResult(false, 
                "Robot stationary but vision poses unstable", 0.0);
        }
        
        // Accept small differences immediately
        if (positionDifference < 0.1 && rotationDifference < 5) {
            return new PoseValidationResult(true, 
                "Small difference from current pose", 1.0);
        }
        
        // For larger differences, scale confidence based on magnitude
        double positionConfidenceScale = Math.max(0.0, 1.0 - (positionDifference / 0.5));
        double rotationConfidenceScale = Math.max(0.0, 1.0 - (rotationDifference / 20.0));
        double combinedScale = Math.min(positionConfidenceScale, rotationConfidenceScale);
        
        // Default validation confidence adjustment
        return new PoseValidationResult(true, 
            "Confidence adjusted for difference magnitude", combinedScale);
    }

    /**
     * Determines if the robot appears to be stationary based on recent pose history
     * 
     * @return True if the robot appears to be stationary
     */
    private boolean isRobotStationary() {
        if (poseHistory.size() < 3) {
            return false;
        }
        
        // Get the last few poses
        int samplesToCheck = Math.min(5, poseHistory.size());
        double maxPositionDelta = 0.0;
        
        Pose2d latestPose = poseHistory.get(poseHistory.size() - 1).pose;
        
        // Check movement over the last few poses
        for (int i = poseHistory.size() - samplesToCheck; i < poseHistory.size() - 1; i++) {
            double delta = poseHistory.get(i).pose.getTranslation()
                .getDistance(latestPose.getTranslation());
            maxPositionDelta = Math.max(maxPositionDelta, delta);
        }
        
        // Consider robot stationary if maximum movement is less than threshold
        return maxPositionDelta < 0.05; // 5 cm threshold
    }

    /**
     * Adds a pose to the history stack, maintaining maximum size
     * 
     * @param pose The pose to add
     * @param timestamp The timestamp of the pose
     */
    private void addPoseToHistory(Pose2d pose, double timestamp) {
        poseHistory.add(new PoseHistoryEntry(pose, timestamp));
        
        // Keep history at maximum size
        if (poseHistory.size() > MAX_POSE_HISTORY_SIZE) {
            poseHistory.remove(0);
        }
    }
    
    /**
     * Update the robot pose using vision measurements from PhotonVision with enhanced rejection logic
     */
    private void updatePoseWithVision() {
        // If vision is disabled or PhotonVision subsystem not initialized, skip
        if (!m_useVision || m_photonVision == null) {
            return;
        }
        
        // Get current pose and add to history for validation purposes
        Pose2d currentPose = getState().Pose;
        addPoseToHistory(currentPose, Timer.getFPGATimestamp());
        
        // Throttle vision updates to avoid overwhelming the pose estimator
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - m_lastVisionUpdateTime < kVisionUpdateRateLimit) {
            return;
        }
        
        // Get the best estimated pose from PhotonVision
        Optional<EstimatedRobotPose> visionEstimate = m_photonVision.getBestEstimatedPose();
        
        if (visionEstimate.isPresent()) {
            EstimatedRobotPose pose = visionEstimate.get();
            
            // Convert Pose3d to Pose2d
            Pose2d visionPose2d = pose.estimatedPose.toPose2d();
            
            // Get the initial confidence in the vision measurement
            double poseConfidence = m_photonVision.calculatePoseConfidence(pose);
            
            // Apply pose validation for better filtering
            PoseValidationResult validationResult = validateVisionPose(
                visionPose2d, currentPose, pose.timestampSeconds);
                
            // Log validation results
            SmartDashboard.putBoolean("Drivetrain/Vision/PoseValid", validationResult.valid);
            SmartDashboard.putString("Drivetrain/Vision/ValidationReason", validationResult.reason);
            SmartDashboard.putNumber("Drivetrain/Vision/ConfidenceScale", validationResult.confidenceScale);
            
            // Adjust confidence based on validation
            poseConfidence *= validationResult.confidenceScale;
            
            // Skip invalid poses
            if (!validationResult.valid) {
                SmartDashboard.putBoolean("Drivetrain/Vision/UpdateAccepted", false);
                SmartDashboard.putString("Drivetrain/Vision/RejectionReason", validationResult.reason);
                return;
            }
            
            // Continue with confidence threshold check
            if (poseConfidence > 0.43) { // Minimum confidence threshold
                // Track consistent vision poses 
                if (m_lastVisionPose == null) {
                    m_lastVisionPose = visionPose2d;
                    m_consistentVisionCount = 1;
                } else {
                    // Check if this vision pose is close to the last one
                    double visionPoseDifference = m_lastVisionPose.getTranslation().getDistance(visionPose2d.getTranslation());
                    double visionRotationDifference = Math.abs(m_lastVisionPose.getRotation().minus(visionPose2d.getRotation()).getDegrees());
                    
                    if (visionPoseDifference < 0.3 && visionRotationDifference < 10) {
                        // Vision pose is consistent with previous one
                        m_consistentVisionCount++;
                    } else {
                        // Vision pose is different, reset counter
                        m_lastVisionPose = visionPose2d;
                        m_consistentVisionCount = 1;
                    }
                    SmartDashboard.putNumber("Drivetrain/Vision/ConsistentCount", m_consistentVisionCount);
                }
                
                // Calculate average distance to AprilTags
                double avgDistance = 0.0;
                for (var target : pose.targetsUsed) {
                    avgDistance += target.getBestCameraToTarget().getTranslation().getNorm();
                }
                avgDistance /= pose.targetsUsed.size();
                
                // Use appropriate standard deviations based on distance
                Matrix<N3, N1> visionStdDevs = avgDistance > kFarVisionThreshold ? 
                    m_farVisionStdDevs : m_defaultVisionStdDevs;
                
                // Apply lower standard deviations (more trust) for multi-tag poses
                if (pose.targetsUsed.size() > 1) {
                    visionStdDevs = visionStdDevs.times(0.7); // 30% lower std devs for multi-tag
                }
                
                // Check if this is an initialization case or robot is disabled
                boolean isInitialization = m_lastVisionUpdateTime == 0 || 
                                          (Math.abs(currentPose.getX()) < 0.1 && 
                                           Math.abs(currentPose.getY()) < 0.1);
                boolean isRobotDisabled = !DriverStation.isEnabled();
                boolean hasConsistentVision = m_consistentVisionCount >= 3; // Require 3 consistent vision poses
                
                // During initialization or when disabled, trust vision more (lower std devs)
                if (isInitialization || isRobotDisabled) {
                    visionStdDevs = visionStdDevs.times(0.5); // 50% lower std devs
                } else if (hasConsistentVision) {
                    // For consistent vision that's far from current pose, use higher std devs
                    // This allows gradual correction instead of jumping
                    double positionDifference = currentPose.getTranslation().getDistance(visionPose2d.getTranslation());
                    if (positionDifference > 0.5) {
                        visionStdDevs = visionStdDevs.times(2.0 + positionDifference);
                        SmartDashboard.putBoolean("Drivetrain/Vision/GradualCorrection", true);
                    }
                }
                
                // Apply the vision measurement to the pose estimator
                addVisionMeasurement(visionPose2d, pose.timestampSeconds, visionStdDevs);
                
                // Save this as the last accepted vision pose
                lastAcceptedVisionPose = visionPose2d;
                lastAcceptedVisionTime = pose.timestampSeconds;
                
                // Log vision update details
                SmartDashboard.putNumber("Drivetrain/Vision/UpdateTime", currentTime);
                SmartDashboard.putNumber("Drivetrain/Vision/Confidence", poseConfidence);
                SmartDashboard.putNumber("Drivetrain/Vision/AvgDistance", avgDistance);
                SmartDashboard.putNumber("Drivetrain/Vision/TagCount", pose.targetsUsed.size());
                SmartDashboard.putBoolean("Drivetrain/Vision/UpdateAccepted", true);
                m_lastVisionUpdateTime = currentTime;
            } else {
                // Log rejected update due to low confidence
                SmartDashboard.putBoolean("Drivetrain/Vision/UpdateAccepted", false);
                SmartDashboard.putString("Drivetrain/Vision/RejectionReason", 
                    "Low confidence: " + String.format("%.2f", poseConfidence));
            }
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
}