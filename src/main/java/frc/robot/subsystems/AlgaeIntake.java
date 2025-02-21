package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeIntake extends SubsystemBase {
    // Physical constants
    private static final double PULLEY_RATIO = 5.0;
    private static final double CURRENT_LIMIT = 40.0;

    // Control constants
    private static final double INTAKE_SPEED = 0.2;
    private static final double REVERSE_SPEED = -1.0;
    private static final double HOLD_SPEED = 0.05;

    // Current spike detection constants
    private static final double CURRENT_SPIKE_THRESHOLD = 20.0; // Amps - adjust based on testing
    private static final double CURRENT_SPIKE_DURATION_THRESHOLD = 0.1; // Seconds
    private static final double CURRENT_DROP_THRESHOLD = 5.0; // Amps - for detecting when ball is lost

    // System state
    private enum IntakeState {
        IDLE,
        INTAKING,
        HOLDING,
        REVERSING,
        ERROR
    }

    // Hardware
    private final TalonFX intakeMotor;
    private final DutyCycleOut dutyCycleControl;
    
    // State tracking
    private IntakeState currentState = IntakeState.IDLE;
    private boolean isBallHeld = false;
    private double spikeStartTime = 0;
    private boolean currentSpikeDetected = false;
    private String errorMessage = "";

    public AlgaeIntake() {
        intakeMotor = new TalonFX(Constants.AlgaeIntakeConstants.algaeIntakeMotorID);
        dutyCycleControl = new DutyCycleOut(0);
        
        try {
            configureMotor();
        } catch (Exception e) {
            handleError("Motor configuration failed: " + e.getMessage());
        }
    }
    
    private void configureMotor() {
        var motorConfig = new TalonFXConfiguration();
        
        // Motor configuration
        motorConfig.Feedback.SensorToMechanismRatio = PULLEY_RATIO;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        
        intakeMotor.getConfigurator().apply(motorConfig);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    
    @Override
    public void periodic() {
        if (currentState != IntakeState.ERROR) {
            try {
                updateBallDetectionState();
                updateMotorState();
                updateDashboard();
            } catch (Exception e) {
                handleError("Periodic update failed: " + e.getMessage());
            }
        }
    }
    
    private void updateMotorState() {
        if (isBallHeld && currentState != IntakeState.REVERSING) {
            setIntakeSpeed(HOLD_SPEED);
            currentState = IntakeState.HOLDING;
        }
    }

    private void updateBallDetectionState() {
        if (currentState == IntakeState.INTAKING) {
            detectBallUsingCurrentSpike();
        } else if (currentState == IntakeState.HOLDING) {
            detectBallLoss();
        }
    }
    
    private void detectBallUsingCurrentSpike() {
        double currentDraw = intakeMotor.getSupplyCurrent().getValueAsDouble();
        
        // Check for current spike when intaking
        if (currentDraw > CURRENT_SPIKE_THRESHOLD && !currentSpikeDetected) {
            currentSpikeDetected = true;
            spikeStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        }
        
        // If current spike persists for the threshold duration, we've got a ball
        if (currentSpikeDetected) {
            double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            if (currentTime - spikeStartTime >= CURRENT_SPIKE_DURATION_THRESHOLD) {
                isBallHeld = true;
                currentState = IntakeState.HOLDING;
                setIntakeSpeed(HOLD_SPEED);
                currentSpikeDetected = false;
            }
        }
    }
    
    private void detectBallLoss() {
        double currentDraw = intakeMotor.getSupplyCurrent().getValueAsDouble();
        
        // If we're holding but current drops significantly, we probably lost the ball
        if (isBallHeld && currentDraw < CURRENT_DROP_THRESHOLD) {
            isBallHeld = false;
            currentState = IntakeState.IDLE;
            setIntakeSpeed(0);
        }
    }
    
    public void intake() {
        if (!isBallHeld && currentState != IntakeState.ERROR) {
            setIntakeSpeed(INTAKE_SPEED);
            currentState = IntakeState.INTAKING;
            currentSpikeDetected = false;
        }
    }
    
    public void reverse() {
        if (currentState != IntakeState.ERROR) {
            setIntakeSpeed(REVERSE_SPEED);
            isBallHeld = false;
            currentState = IntakeState.REVERSING;
            currentSpikeDetected = false;
        }
    }
    
    public void stop() {
        if (currentState != IntakeState.ERROR) {
            setIntakeSpeed(0);
            currentState = IntakeState.IDLE;
            currentSpikeDetected = false;
        }
    }
    
    private void setIntakeSpeed(double speed) {
        try {
            intakeMotor.setControl(dutyCycleControl.withOutput(speed));
        } catch (Exception e) {
            handleError("Failed to set motor speed: " + e.getMessage());
        }
    }
    
    private void handleError(String message) {
        currentState = IntakeState.ERROR;
        errorMessage = message;
        try {
            intakeMotor.stopMotor();
        } catch (Exception e) {
            // If we can't even stop the motor, log it but don't throw
            System.err.println("Failed to stop motor in error handler: " + e.getMessage());
        }
    }
    
    public boolean hasBall() {
        return isBallHeld;
    }
    
    private void updateDashboard() {
        SmartDashboard.putBoolean("Algae/Has Ball", isBallHeld);
        SmartDashboard.putString("Algae/State", currentState.toString());
        SmartDashboard.putNumber("Algae/Current Draw", 
            intakeMotor.getSupplyCurrent().getValueAsDouble());
        
        if (currentState == IntakeState.ERROR) {
            SmartDashboard.putString("Algae/Error", errorMessage);
        }
    }

    public boolean isOperational() {
        return currentState != IntakeState.ERROR;
    }
}