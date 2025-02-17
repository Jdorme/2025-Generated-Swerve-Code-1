package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {
    // Network tables for both Limelights
    private final NetworkTable m_limelightFront;
    private final NetworkTable m_limelightBack;
    
    // Track connection status for each Limelight
    private boolean m_frontConnected = false;
    private boolean m_backConnected = false;
    
    // Track last successful connection attempt time
    private double m_lastFrontConnectionAttempt = 0;
    private double m_lastBackConnectionAttempt = 0;
    
    // How often to retry connecting to disconnected Limelights (seconds)
    private static final double CONNECTION_RETRY_INTERVAL = 1.0;
    
    // Minimum confidence score to accept a vision measurement
    private static final double MIN_CONFIDENCE = 0.75;
    
    // Reference to the drivetrain for updating odometry
    private final CommandSwerveDrivetrain m_drivetrain;
    
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        
        // Initialize NetworkTables connections to both Limelights
        m_limelightFront = NetworkTableInstance.getDefault().getTable("limelight-front");
        m_limelightBack = NetworkTableInstance.getDefault().getTable("limelight-back");
        
        // Initial connection attempt
        checkAndConfigureLimelight(m_limelightFront, "Front");
        checkAndConfigureLimelight(m_limelightBack, "Back");
    }
    
    private boolean checkAndConfigureLimelight(NetworkTable limelightTable, String name) {
        try {
            // Try to read a value to check connection
            NetworkTableEntry pipelineEntry = limelightTable.getEntry("pipeline");
            pipelineEntry.getDouble(-1);
            
            // If we get here, connection was successful
            configureLimelight(limelightTable);
            SmartDashboard.putString("Vision/" + name + "/Status", "Connected");
            return true;
        } catch (Exception e) {
            SmartDashboard.putString("Vision/" + name + "/Status", "Disconnected");
            return false;
        }
    }
    public void processSimulatedMeasurement(Pose2d simulatedPose) {
    // In simulation, bypass the Limelight network tables and feed the pose directly
    if (RobotBase.isSimulation()) {
        // Get current time
        double currentTime = Timer.getFPGATimestamp();
        
        // Feed the simulated measurement to the drivetrain
        m_drivetrain.addVisionMeasurement(simulatedPose, currentTime);
        
        // Log the simulated measurement to SmartDashboard
        SmartDashboard.putString("Vision/Simulated Pose", 
            String.format("(%.2f, %.2f) %.2f°", 
                simulatedPose.getX(), 
                simulatedPose.getY(), 
                simulatedPose.getRotation().getDegrees()));
    }
}
    private void configureLimelight(NetworkTable limelightTable) {
        try {
            // Set pipeline to 0 (AprilTag pipeline)
            limelightTable.getEntry("pipeline").setNumber(0);
            // Set to vision processing mode
            limelightTable.getEntry("camMode").setNumber(0);
        } catch (Exception e) {
            // If configuration fails, we'll try again next connection attempt
        }
    }
    
    // Connection checking is handled in periodic()
    
    @Override
    public void periodic() {
        double currentTime = Timer.getFPGATimestamp();
        
        // Check front Limelight connection
        if (!m_frontConnected && currentTime - m_lastFrontConnectionAttempt >= CONNECTION_RETRY_INTERVAL) {
            m_frontConnected = checkAndConfigureLimelight(m_limelightFront, "Front");
            m_lastFrontConnectionAttempt = currentTime;
        }
        
        // Check back Limelight connection
        if (!m_backConnected && currentTime - m_lastBackConnectionAttempt >= CONNECTION_RETRY_INTERVAL) {
            m_backConnected = checkAndConfigureLimelight(m_limelightBack, "Back");
            m_lastBackConnectionAttempt = currentTime;
        }
        
        // Only process data from connected Limelights
        if (m_frontConnected) {
            processLimelightData(m_limelightFront, "Front");
        }
        if (m_backConnected) {
            processLimelightData(m_limelightBack, "Back");
        }
    }
    
    private void processLimelightData(NetworkTable limelight, String name) {
        try {
            // Check if we have a valid target
            double valid = limelight.getEntry("tv").getDouble(0.0);
            if (valid < 1.0) {
                return;  // No valid target
            }
            
            // Get the confidence score
            double confidence = limelight.getEntry("ta").getDouble(0.0);
            if (confidence < MIN_CONFIDENCE) {
                return;  // Low confidence measurement
            }
            
            // Get robot pose from Limelight
            double[] botpose = limelight.getEntry("botpose").getDoubleArray(new double[6]);
            if (botpose.length < 6) {
                return;  // Invalid pose data
            }
            
            // Create Pose2d from Limelight data
            Pose2d visionPose = new Pose2d(
                botpose[0], 
                botpose[1],
                Rotation2d.fromDegrees(botpose[5])
            );
            
            // Get the capture timestamp
            double timestamp = Timer.getFPGATimestamp() - (botpose[6] / 1000.0);
            
            // Use the drivetrain's built-in vision measurement system
            m_drivetrain.addVisionMeasurement(visionPose, timestamp);
            
            // Log data
            SmartDashboard.putString("Vision/" + name + "/Robot Pose", 
                String.format("(%.2f, %.2f) %.2f°", 
                    visionPose.getX(), 
                    visionPose.getY(), 
                    visionPose.getRotation().getDegrees()));
            SmartDashboard.putNumber("Vision/" + name + "/Confidence", confidence);
            
        } catch (Exception e) {
            // If we get any exception while processing data, mark the Limelight as disconnected
            if (name.equals("Front")) {
                m_frontConnected = false;
            } else {
                m_backConnected = false;
            }
            SmartDashboard.putString("Vision/" + name + "/Status", "Error: " + e.getMessage());
        }
    }
    
    public boolean isFrontLimelightConnected() {
        return m_frontConnected;
    }
    
    public boolean isBackLimelightConnected() {
        return m_backConnected;
    }
}