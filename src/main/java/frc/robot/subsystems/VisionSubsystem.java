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
    private final NetworkTable m_limelightSource;
    private final NetworkTable m_limelightReef;
    
    // Track connection status for each Limelight
    private boolean m_sourceConnected = false;
    private boolean m_reefConnected = false;
    
    // Track last successful connection attempt time
    private double m_lastSourceConnectionAttempt = 0;
    private double m_lastReefConnectionAttempt = 0;
    
    // How often to retry connecting to disconnected Limelights (seconds)
    private static final double CONNECTION_RETRY_INTERVAL = 1.0;
    
    // Minimum confidence score to accept a vision measurement
    private static final double MIN_CONFIDENCE = 0.75;
    
    // Reference to the drivetrain for updating odometry
    private final CommandSwerveDrivetrain m_drivetrain;
    
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        
        // Initialize NetworkTables connections to both Limelights
        m_limelightSource = NetworkTableInstance.getDefault().getTable("source");
        m_limelightReef = NetworkTableInstance.getDefault().getTable("reef");
        
        // Initial connection attempt
        checkAndConfigureLimelight(m_limelightSource, "Source");
        checkAndConfigureLimelight(m_limelightReef, "Reef");
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
    
    @Override
    public void periodic() {
        double currentTime = Timer.getFPGATimestamp();
        
        // Check Source Limelight connection
        if (!m_sourceConnected && currentTime - m_lastSourceConnectionAttempt >= CONNECTION_RETRY_INTERVAL) {
            m_sourceConnected = checkAndConfigureLimelight(m_limelightSource, "Source");
            m_lastSourceConnectionAttempt = currentTime;
        }
        
        // Check Reef Limelight connection
        if (!m_reefConnected && currentTime - m_lastReefConnectionAttempt >= CONNECTION_RETRY_INTERVAL) {
            m_reefConnected = checkAndConfigureLimelight(m_limelightReef, "Reef");
            m_lastReefConnectionAttempt = currentTime;
        }
        
        // Only process data from connected Limelights
        if (m_sourceConnected) {
            processLimelightData(m_limelightSource, "Source");
        }
        if (m_reefConnected) {
            processLimelightData(m_limelightReef, "Reef");
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
            if (name.equals("Source")) {
                m_sourceConnected = false;
            } else {
                m_reefConnected = false;
            }
            SmartDashboard.putString("Vision/" + name + "/Status", "Error: " + e.getMessage());
        }
    }
    
    public boolean isSourceLimelightConnected() {
        return m_sourceConnected;
    }
    
    public boolean isReefLimelightConnected() {
        return m_reefConnected;
    }
}