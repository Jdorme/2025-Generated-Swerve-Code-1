package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private boolean odometrySeeded = false;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* SysId routines (unchanged) */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null,
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(7), null,
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
        new SysIdRoutine.Mechanism(volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(Math.PI / 6).per(Second), Volts.of(Math.PI), null,
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null, this));

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    // ---------------- Limelight vision state ----------------
    private PoseEstimate farmOne, farmTwo, bestPose;
    private boolean useMegaTag2 = true;   // switch between pipeline modes
    private double angularVelocity = 0.0;  // update from gyro if needed

    // ---------------- Constructors ----------------
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
                                   SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) startSimThread();
        configureAutoBuilder();
        setupLimelights();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
                                   double odometryUpdateFrequency,
                                   SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) startSimThread();
        configureAutoBuilder();
        setupLimelights();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
                                   double odometryUpdateFrequency,
                                   Matrix<N3, N1> odometryStandardDeviation,
                                   Matrix<N3, N1> visionStandardDeviation,
                                   SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) startSimThread();
        configureAutoBuilder();
        setupLimelights();
    }

    // ---------------- Limelight Setup ----------------
    public void setupLimelights() {
        try {
            // Initialize both Limelights
            LimelightHelpers.setPipelineIndex("limelight-elevato", 0);
            LimelightHelpers.setLEDMode_PipelineControl("limelight-elevato");
            //LimelightHelpers.setCameraMode_Driver("limelight-elevato", false);
            LimelightHelpers.setPipelineIndex("limelight-climber", 0);
            LimelightHelpers.setLEDMode_PipelineControl("limelight-climber");
            //LimelightHelpers.setCameraMode_Driver("limelight-climber", false);
            SmartDashboard.putString("Limelight Setup", "OK");
        } catch (Exception e) {
            SmartDashboard.putString("Limelight Setup", "Error initializing Limelights");
        }
        SmartDashboard.putBoolean("Use MegaTag2", useMegaTag2);
        SmartDashboard.putBoolean("Enable Vision", true);
    }

    // ---------------- Limelight Mode Update ----------------
    public void updateLimelightMode() {
        boolean megaTag2Enabled = SmartDashboard.getBoolean("Use MegaTag2", useMegaTag2);
        boolean visionEnabled = SmartDashboard.getBoolean("Enable Vision", true);
        useMegaTag2 = megaTag2Enabled;
        if (!visionEnabled) return;
        
        // CRITICAL: Use the fused pose estimate rotation, NOT raw gyro
        // This includes both odometry and vision corrections
        double yawDegrees = getState().Pose.getRotation().getDegrees();
        
        // For angular velocity, raw gyro is fine since it's instantaneous
        double yawRate = getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
        
        // Get pitch and roll from gyro (these don't need fusion for swerve)
        double pitchDegrees = getPigeon2().getPitch().getValueAsDouble();
        double rollDegrees = getPigeon2().getRoll().getValueAsDouble();
        SmartDashboard.putNumber("Robot Heading (deg)", getState().Pose.getRotation().getDegrees());
        
        // Set robot orientation for both Limelights
        LimelightHelpers.SetRobotOrientation("limelight-elevato", 
            yawDegrees, yawRate, 
            pitchDegrees, 0,
            rollDegrees, 0);
            
        LimelightHelpers.SetRobotOrientation("limelight-climber", 
            yawDegrees, yawRate, 
            pitchDegrees, 0, 
            rollDegrees, 0);
        
        // Debug output
        SmartDashboard.putNumber("Fused Rotation (deg)", yawDegrees);
        SmartDashboard.putNumber("Raw Gyro (deg)", 
        MathUtil.inputModulus(getPigeon2().getRotation2d().getDegrees(), 0, 360));
        SmartDashboard.putNumber("Angular Velocity (deg/s)", yawRate);
    }

    // ---------------- Limelight Update ----------------
    public void updateOdometryWithVision() {
        boolean visionPoseAccepted = false; 
    
        
        // --- Query both Limelights with appropriate pipeline mode ---
        PoseEstimate farmOneForSeeding = null;
        PoseEstimate farmTwoForSeeding = null;
        
        if (!odometrySeeded) {
            // For seeding, always use non-MegaTag2 to get better heading
            farmOneForSeeding = safeGetPose("limelight-elevato", false);  // false = non-MegaTag2
            farmTwoForSeeding = safeGetPose("limelight-climber", false);  // false = non-MegaTag2
        }
        
        // For regular updates, use the configured pipeline mode
        farmOne = safeGetPose("limelight-elevato", useMegaTag2);
        farmTwo = safeGetPose("limelight-climber", useMegaTag2);
    
        // --- Debug raw values ---
        if (farmOne != null) {
            SmartDashboard.putNumber("Eleavto_raw_tagCount", farmOne.tagCount);
            SmartDashboard.putNumber("Eleavto_raw_X", farmOne.pose.getX());
            SmartDashboard.putNumber("Eleavto_raw_Y", farmOne.pose.getY());
        } else {
            SmartDashboard.putString("Eleavto_raw", "null");
        }
    
        if (farmTwo != null) {
            SmartDashboard.putNumber("Climber_raw_tagCount", farmTwo.tagCount);
            SmartDashboard.putNumber("Climber_raw_X", farmTwo.pose.getX());
            SmartDashboard.putNumber("Climber_raw_Y", farmTwo.pose.getY());
        } else {
            SmartDashboard.putString("Climber_raw", "null");
        }
    
        // --- If first valid vision, seed odometry with non-MegaTag2 pose ---
        if (!odometrySeeded) {
            boolean rejectOneSeeding = shouldReject(farmOneForSeeding, "limelight-elevato");
            boolean rejectTwoSeeding = shouldReject(farmTwoForSeeding, "limelight-climber");
            
            PoseEstimate seedingPose = pickBestPose(farmOneForSeeding, farmTwoForSeeding, rejectOneSeeding, rejectTwoSeeding);
            
            if (seedingPose != null) {
                SmartDashboard.putString("Seeding Camera", 
                    seedingPose == farmOneForSeeding ? "limelight-elevato" : "limelight-climber");
                SmartDashboard.putNumber("Seeding X", seedingPose.pose.getX());
                SmartDashboard.putNumber("Seeding Y", seedingPose.pose.getY());
                SmartDashboard.putNumber("Seeding Heading (deg)", seedingPose.pose.getRotation().getDegrees());
                SmartDashboard.putNumber("Seeding Tag Count", seedingPose.tagCount);
                
                resetPose(seedingPose.pose); // reset odometry with non-MegaTag2 pose (better heading)
                odometrySeeded = true;
                SmartDashboard.putString("Odometry Seed", "Seeding done with non-MegaTag2");
                
                // Don't do regular vision fusion this cycle since we just seeded
                return;
            }
        }
    
        // --- Regular vision fusion (after seeding) ---
        if (odometrySeeded) {
            // Reject bad estimates for regular updates
            boolean rejectOne = shouldReject(farmOne, "limelight-elevato");
            boolean rejectTwo = shouldReject(farmTwo, "Limelight-climber");

            // Pick best pose for regular updates
            bestPose = pickBestPose(farmOne, farmTwo, rejectOne, rejectTwo);
            
            // Determine which Limelight was selected
            String bestLimelight = null;
            if (bestPose == farmOne) bestLimelight = "limelight-elevato";
            else if (bestPose == farmTwo) bestLimelight = "limelight-climber";

            // Fuse if valid
            if (bestPose != null && bestLimelight != null) {
                double latencySec = LimelightHelpers.getLatency_Pipeline(bestLimelight) / 1000.0;
                double correctedTimestamp = Utils.getCurrentTimeSeconds() - latencySec;
                setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
                addVisionMeasurement(bestPose.pose, correctedTimestamp);
                visionPoseAccepted = true;
                
                // Debug output
                double now = Utils.getCurrentTimeSeconds();
                SmartDashboard.putNumber("Vision Delta Time (s)", now - correctedTimestamp);
            }
        }
    
        // --- Publish status ---
        SmartDashboard.putBoolean("Vision Pose Accepted", visionPoseAccepted);
        SmartDashboard.putBoolean("Odometry Seeded", odometrySeeded);
    
        if (bestPose != null) {
            SmartDashboard.putNumber("Vision Tag Count", bestPose.tagCount);
            SmartDashboard.putNumber("Vision X", bestPose.pose.getX());
            SmartDashboard.putNumber("Vision Y", bestPose.pose.getY());
            SmartDashboard.putNumber("Vision Heading", bestPose.pose.getRotation().getDegrees());
        } else {
            SmartDashboard.putString("Vision Status", "no bestPose");
        }
    }

    private PoseEstimate safeGetPose(String limelight, boolean megaTag2) {
        try {
            // Always use Blue origin regardless of alliance
            // The swerve drivetrain handles alliance flipping internally
            return megaTag2
                ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
        } catch (Exception e) {
            return null;
        }
    }

    private boolean shouldReject(PoseEstimate est, String name) {
        if (est == null) {
            SmartDashboard.putString(name + " Status", "Disconnected");
            return true;
        }
        SmartDashboard.putString(name + " Status", "Ready");

        if (est.tagCount == 0) return true;
        
        // Reject if spinning too fast (from Limelight example)
        double angularVelocity = Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
        if (angularVelocity > 360) {
            SmartDashboard.putString(name + " Reject Reason", "Spinning too fast");
            return true;
        }
        
        if (est.tagCount == 1 && est.rawFiducials.length == 1) {
            // Add debug output for both cameras
            SmartDashboard.putNumber(name + " Ambiguity", est.rawFiducials[0].ambiguity);
            SmartDashboard.putNumber(name + " Distance", est.rawFiducials[0].distToCamera);
            
            if (est.rawFiducials[0].ambiguity > 0.7) {
                SmartDashboard.putString(name + " Reject Reason", "High ambiguity");
                return true;
            }
            if (est.rawFiducials[0].distToCamera > 3.0) {




                
                SmartDashboard.putString(name + " Reject Reason", "Too far");
                return true;
            }
        }
        
        SmartDashboard.putString(name + " Reject Reason", "Accepted");
        return false;
    }

    private PoseEstimate pickBestPose(PoseEstimate one, PoseEstimate two, boolean rejectOne, boolean rejectTwo) {
        if (!rejectOne && !rejectTwo) {
            return (one.avgTagArea >= two.avgTagArea) ? one : two;
        } else if (!rejectOne) {
            return one;
        } else if (!rejectTwo) {
            return two;
        }
        return null;
    }
    

    // ---------------- PathPlanner ----------------
    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                new PPHolonomicDriveController(
                    new PIDConstants(10, 0, 0),
                    new PIDConstants(7, 0, 0)),
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /** SysId Quasistatic */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /** SysId Dynamic */
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

        // Update Limelight orientation before vision update
        updateLimelightMode();
        
        // Auto-update vision each cycle
        updateOdometryWithVision();
        
        SmartDashboard.putNumber("Robot X (m)", getState().Pose.getX());

        
        SmartDashboard.putNumber("Robot Y (m)", getState().Pose.getY());
        SmartDashboard.putNumber("Robot Heading (deg)", getState().Pose.getRotation().getDegrees());
        
        if (bestPose != null) {
            SmartDashboard.putNumber("Vision Tag Count", bestPose.tagCount);
            SmartDashboard.putNumber("Vision X", bestPose.pose.getX());
            SmartDashboard.putNumber("Vision Y", bestPose.pose.getY());
            SmartDashboard.putNumber("Vision Heading", bestPose.pose.getRotation().getDegrees());
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
}