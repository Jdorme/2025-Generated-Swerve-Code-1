package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    // Physical constants
    private static final double GEAR_RATIO = 9.0; // 9:1 reduction
    private static final double SPROCKET_PITCH_DIAMETER = 1.751; // inches
    private static final double MAX_HEIGHT = 29.7; // inches
    
    // Motors
    private final TalonFX masterMotor;
    private final TalonFX followerMotor;
    
    private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0).withSlot(0);
    
    // Target position tracking
    private double targetPositionInches = 0.0;
    
    public ElevatorSubsystem() {
        masterMotor = new TalonFX(Constants.ElevatorConstants.elevatorMotorRID);
        followerMotor = new TalonFX(Constants.ElevatorConstants.elevatorMotorLID);
        
        // Configure both motors
        configureMotors();
    }
    
    private void configureMotors() {
        var motorConfig = new TalonFXConfiguration();
        
        // Basic motor configuration
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Current limits for safety
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Configure the PID gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 15; // Increased from 1.0
        slot0Configs.kI = 0; // Added some integral gain
        slot0Configs.kD = 0; // Added some derivative gain
        slot0Configs.kV = 0; // Added feed forward
        motorConfig.Slot0 = slot0Configs;
        
        // Motion Magic settings
        var motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.MotionMagicConstants.ElevatorMotionMagic.getCruiseVelocity();
        motionMagicConfigs.MotionMagicAcceleration = Constants.MotionMagicConstants.ElevatorMotionMagic.getAcceleration();
        motionMagicConfigs.MotionMagicJerk = Constants.MotionMagicConstants.ElevatorMotionMagic.getJerk();
        motorConfig.MotionMagic = motionMagicConfigs;
        
        // Configure master motor
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        masterMotor.getConfigurator().apply(motorConfig);
        
        // Configure follower motor
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        followerMotor.getConfigurator().apply(motorConfig);
        
        // Keep follower following in same direction as master
        followerMotor.setControl(new Follower(masterMotor.getDeviceID(), true));
    }
    
    public void setHeight(double heightInches) {
        // Clamp height to valid range
        targetPositionInches = Math.min(Math.max(heightInches, 0), MAX_HEIGHT);
        
        // Convert inches to rotations
        double sprocketRotations = targetPositionInches / (SPROCKET_PITCH_DIAMETER * Math.PI);
        double motorRotations = sprocketRotations * GEAR_RATIO;
        
        masterMotor.setControl(motionRequest.withPosition(motorRotations));
    }
    
    public double getCurrentHeight() {
        // Get motor rotations
        double motorRotations = masterMotor.getPosition().getValueAsDouble();
        
        // Convert rotations to inches
        double sprocketRotations = motorRotations / GEAR_RATIO;
        return sprocketRotations * (SPROCKET_PITCH_DIAMETER * Math.PI);
    }
    
    public boolean isAtTarget() {
        return Math.abs(getErrorInches()) < 0.1;
    }
    
    public double getErrorInches() {
        // Get the closed-loop error in rotations
        double errorRotations = masterMotor.getClosedLoopError().getValueAsDouble();
        
        // Convert rotational error to sprocket rotations
        double sprocketRotations = errorRotations / GEAR_RATIO;
        
        // Convert sprocket rotations to linear inches
        return sprocketRotations * (SPROCKET_PITCH_DIAMETER * Math.PI);
    }
    
    public void stop() {
        masterMotor.stopMotor();
    }
    
    public void setMotorOutput(double percentOutput) {
        masterMotor.set(percentOutput);
    }

    @Override
    public void periodic() {
        // Log elevator information to SmartDashboard
        SmartDashboard.putNumber("Elevator/Current Height", getCurrentHeight());
        SmartDashboard.putNumber("Elevator/Target Position", targetPositionInches);
        SmartDashboard.putNumber("Elevator/Position Error", getErrorInches());
        SmartDashboard.putBoolean("At Target", isAtTarget());
    }
}