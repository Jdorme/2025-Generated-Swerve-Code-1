package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    // Physical constants
    private static final double GEAR_RATIO = 5.0; // 5:1 reduction
    private static final double SPROCKET_PITCH_DIAMETER = 1.751; // inches
    private static final double MAX_HEIGHT = 25.5; // inches
    
    // Motors
    private final TalonFX masterMotor;
    private final TalonFX followerMotor;
    
    // CANCoder
    private final CANcoder elevatorEncoder;
    
    private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0).withSlot(0);
    
    // Target position tracking
    private double targetPositionInches = 0.0;
    
    public ElevatorSubsystem() {
        masterMotor = new TalonFX(Constants.ElevatorConstants.elevatorMotorRID);
        followerMotor = new TalonFX(Constants.ElevatorConstants.elevatorMotorLID);
        elevatorEncoder = new CANcoder(Constants.ElevatorConstants.elevatorEncoderID);
        
        // Configure the CANCoder
        configureCANCoder();
        
        // Configure both motors
        configureMotors();
        
        // Zero the encoder at initialization
        zeroEncoder();
    }
    
    private void configureCANCoder() {
        var encoderConfig = new CANcoderConfiguration();
        
        // Configure sensor direction if needed
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        
        // Apply configuration
        elevatorEncoder.getConfigurator().apply(encoderConfig);
    }
    
    private void configureMotors() {
        var motorConfig = new TalonFXConfiguration();
        
        // Basic motor configuration
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Current limits for safety
        motorConfig.CurrentLimits.SupplyCurrentLimit = 60;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Configure the PID gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 15; // Increased from 1.0
        slot0Configs.kI = 0; // Added some integral gain
        slot0Configs.kD = 0; // Added some derivative gain
        slot0Configs.kV = 0; // Added feed forward
        motorConfig.Slot0 = slot0Configs;
        
        // Motion Magic settings - ADJUSTED FOR CANCODER
        // Since we're now measuring in sprocket rotations instead of motor rotations,
        // we need to divide our motion magic settings by the gear ratio.
        // This ensures the actual speed of the elevator remains the same.
        var motionMagicConfigs = new MotionMagicConfigs();
        
        // Get the original values that worked with the integrated encoder
        double originalVelocity = Constants.MotionMagicConstants.ElevatorMotionMagic.getCruiseVelocity();
        double originalAcceleration = Constants.MotionMagicConstants.ElevatorMotionMagic.getAcceleration();
        double originalJerk = Constants.MotionMagicConstants.ElevatorMotionMagic.getJerk();
        
        // Adjust for sprocket rotations (divide by gear ratio)
        motionMagicConfigs.MotionMagicCruiseVelocity = originalVelocity / GEAR_RATIO;
        motionMagicConfigs.MotionMagicAcceleration = originalAcceleration / GEAR_RATIO;
        motionMagicConfigs.MotionMagicJerk = originalJerk / GEAR_RATIO;
        
        motorConfig.MotionMagic = motionMagicConfigs;
        
        // Configure feedback to use remote CANCoder
        var feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.FeedbackRemoteSensorID = elevatorEncoder.getDeviceID();
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // Since CANCoder is directly on the sprocket, we don't need to account for gear ratio in the feedback
        feedbackConfigs.SensorToMechanismRatio = 1.0;
        motorConfig.Feedback = feedbackConfigs;
        
        // Configure master motor
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        masterMotor.getConfigurator().apply(motorConfig);
        
        // Configure follower motor
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        followerMotor.getConfigurator().apply(motorConfig);
        
        // Keep follower following in same direction as master
        followerMotor.setControl(new Follower(masterMotor.getDeviceID(), true));
    }
    
    public void setHeight(double heightInches) {
        // Clamp height to valid range
        targetPositionInches = Math.min(Math.max(heightInches, 0), MAX_HEIGHT);
        
        // Convert inches to rotations (of the sprocket)
        double sprocketRotations = targetPositionInches / (SPROCKET_PITCH_DIAMETER * Math.PI);
        
        // Set the target position in terms of sprocket rotations
        masterMotor.setControl(motionRequest.withPosition(sprocketRotations));
    }
    
    public double getCurrentHeight() {
        // Get sprocket rotations directly from the CANCoder
        double sprocketRotations = elevatorEncoder.getPosition().getValueAsDouble();
        
        // Convert sprocket rotations to linear distance in inches
        return sprocketRotations * (SPROCKET_PITCH_DIAMETER * Math.PI);
    }
    
    public boolean isAtTarget() {
        return Math.abs(getErrorInches()) < 0.1;
    }
    
    public double getErrorInches() {
        // Get the closed-loop error in rotations (of the sprocket)
        double errorRotations = masterMotor.getClosedLoopError().getValueAsDouble();
        
        // Convert sprocket rotations to linear inches
        return errorRotations * (SPROCKET_PITCH_DIAMETER * Math.PI);
    }
    
    public void stop() {
        masterMotor.stopMotor();
    }
    
    public void setMotorOutput(double percentOutput) {
        masterMotor.set(percentOutput);
    }
    
    /**
     * Zero the encoder at the current position.
     * Call this when the elevator is at a known physical position.
     */
    public void zeroEncoder() {
        // Zero the encoder at the current position
        elevatorEncoder.setPosition(0);
        
        // Also update the target position to match
        targetPositionInches = 0.0;
        
        System.out.println("Elevator encoder zeroed");
    }
    
    /**
     * Zero the encoder at a specified height.
     * Useful for calibration if the elevator is at a known non-zero position.
     * 
     * @param initialHeightInches The current height of the elevator in inches
     */
    public void zeroEncoderAtHeight(double initialHeightInches) {
        // Calculate rotations for the specified height
        double rotations = initialHeightInches / (SPROCKET_PITCH_DIAMETER * Math.PI);
        
        // Set the encoder to this value
        elevatorEncoder.setPosition(rotations);
        
        // Set target position to match
        targetPositionInches = initialHeightInches;
        
        System.out.println("Elevator encoder zeroed at height: " + initialHeightInches + " inches");
    }

    @Override
    public void periodic() {
        // Log elevator information to SmartDashboard
        SmartDashboard.putNumber("Elevator/Current Height", getCurrentHeight());
        SmartDashboard.putNumber("Elevator/Target Position", targetPositionInches);
        SmartDashboard.putNumber("Elevator/Position Error", getErrorInches());
        SmartDashboard.putNumber("Elevator/CANCoder Position", elevatorEncoder.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Elevator/At Target", isAtTarget());
    }
}