package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeIntake extends SubsystemBase {
    private final TalonFX intakeMotor;
    
    // Constants
    private static final double INTAKE_SPEED = 0.7;
    private static final double HOLD_TORQUE = 2.0; // Nm
    private static final double CURRENT_SPIKE_THRESHOLD = 40.0; // Increased from 30A to 40A
    private static final int CURRENT_SPIKE_DURATION_THRESHOLD = 10; // Increased from 5 to 10 cycles
    private static final double CURRENT_FILTER_ALPHA = 0.1; // For current smoothing
    
    // Control requests
    private final TorqueCurrentFOC torqueRequest;
    private final VelocityVoltage velocityRequest;
    
    // Current spike detection
    private int currentSpikeCounter = 0;
    private boolean hasBall = false;
    private double filteredCurrent = 0;
    private boolean isIntaking = false;
    
    public AlgaeIntake() {
        // Initialize Kraken X60
        intakeMotor = new TalonFX(1); // Change ID as needed
        
        // Configure motor
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.Slot0.kP = 0.11;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        
        intakeMotor.getConfigurator().apply(configs);
        
        // Initialize control requests
        torqueRequest = new TorqueCurrentFOC(0);
        velocityRequest = new VelocityVoltage(0);
    }
    
    public void intakeBall() {
        isIntaking = true;
        intakeMotor.setControl(velocityRequest.withVelocity(INTAKE_SPEED * 100)); // Convert to RPS
    }
    
    public void holdBall() {
        isIntaking = false;
        intakeMotor.setControl(torqueRequest.withOutput(HOLD_TORQUE));
    }
    
    public void stop() {
        isIntaking = false;
        intakeMotor.setControl(velocityRequest.withVelocity(0));
        hasBall = false;
        currentSpikeCounter = 0;
        filteredCurrent = 0;
    }
    
    public boolean hasBall() {
        return hasBall;
    }
    
    @Override
    public void periodic() {
        if (!isIntaking) {
            return; // Don't check for current spikes if we're not actively intaking
        }
        
        // Get and filter current
        double rawCurrent = intakeMotor.getStatorCurrent().getValueAsDouble();
        filteredCurrent = (CURRENT_FILTER_ALPHA * rawCurrent) + ((1 - CURRENT_FILTER_ALPHA) * filteredCurrent);
        
        // Output debug values to SmartDashboard
        SmartDashboard.putNumber("Intake Raw Current", rawCurrent);
        SmartDashboard.putNumber("Intake Filtered Current", filteredCurrent);
        SmartDashboard.putNumber("Current Spike Counter", currentSpikeCounter);
        SmartDashboard.putBoolean("Has Ball", hasBall);
        
        if (filteredCurrent >= CURRENT_SPIKE_THRESHOLD) {
            currentSpikeCounter++;
            if (currentSpikeCounter >= CURRENT_SPIKE_DURATION_THRESHOLD && !hasBall) {
                hasBall = true;
                holdBall(); // Switch to torque control once ball is detected
            }
        } else {
            currentSpikeCounter = Math.max(0, currentSpikeCounter - 2); // Gradual decrease
        }
    }
}