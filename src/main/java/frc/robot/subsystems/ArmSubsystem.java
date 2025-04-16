package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    // Physical constants
    private static final double GEAR_RATIO = 74.5; // 102.5:1 reduction changed to 74.5 16t sprocket to 22t
    private static final double MAX_ANGLE = 180.0; // degrees
    private static final double MIN_ANGLE = -210; // degrees - new minimum angle
    
    // Motor
    private final TalonFX motor;
    private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0).withSlot(0);
    
    // Target position tracking
    private double targetPositionDegrees = 0.0;
    
    public ArmSubsystem() {
        motor = new TalonFX(Constants.ArmConstants.armMotorID);
        configureMotor();
    }
    
    private void configureMotor() {
        var motorConfig = new TalonFXConfiguration();
        
        // Basic motor configuration
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Current limits for safety
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Configure the PID gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 15;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.1;
        slot0Configs.kV = 0.12;
        motorConfig.Slot0 = slot0Configs;
        
        // Motion Magic settings
        var motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.MotionMagicConstants.ArmMotionMagic.getCruiseVelocity();
        motionMagicConfigs.MotionMagicAcceleration = Constants.MotionMagicConstants.ArmMotionMagic.getAcceleration();
        motionMagicConfigs.MotionMagicJerk = Constants.MotionMagicConstants.ArmMotionMagic.getJerk();
        motorConfig.MotionMagic = motionMagicConfigs;
        
        // Configure motor
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motor.getConfigurator().apply(motorConfig);
    }
    
    public void setAngle(double angleDegrees) {
        // Clamp angle to valid range between MIN_ANGLE and MAX_ANGLE
        targetPositionDegrees = Math.min(Math.max(angleDegrees, MIN_ANGLE), MAX_ANGLE);
        
        // Convert degrees to motor rotations
        double motorRotations = (targetPositionDegrees / 360.0) * GEAR_RATIO;
        
        motor.setControl(motionRequest.withPosition(motorRotations));
    }
    
    public double getCurrentAngle() {
        // Get motor rotations and convert to degrees
        double motorRotations = motor.getPosition().getValueAsDouble();
        return (motorRotations / GEAR_RATIO) * 360.0;
    }
    
    public boolean isAtTarget() {
        return Math.abs(getErrorDegrees()) < 2.0;
    }
    
    public double getErrorDegrees() {
        // Get the closed-loop error in rotations
        double errorRotations = motor.getClosedLoopError().getValueAsDouble();
        return (errorRotations / GEAR_RATIO) * 360.0;
    }
    
    public void stop() {
        motor.stopMotor();
    }
    
    public void setMotorOutput(double percentOutput) {
        motor.set(percentOutput);
    }

    @Override
    public void periodic() {
        // Log arm information to SmartDashboard
        SmartDashboard.putNumber("Arm/Current Angle", getCurrentAngle());
        SmartDashboard.putNumber("Arm/Target Angle", targetPositionDegrees);
        SmartDashboard.putNumber("Arm/Position Error", getErrorDegrees());
        SmartDashboard.putBoolean("Arm/At Target", isAtTarget());
    }
}