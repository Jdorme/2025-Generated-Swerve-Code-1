package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeIntake extends SubsystemBase {
    // Physical constants
    private static final double PULLEY_RATIO = 5.0;
    private static final double CURRENT_LIMIT = 40.0;

    // Control constants
    private static final double INTAKE_SPEED = 0.7;
    private static final double REVERSE_SPEED = -1.0;
    private static final double HOLD_SPEED = 0.05;

    // Sensor thresholds
    private static final double BALL_DETECTION_THRESHOLD = 0.2;
    private static final double AMBIENT_THRESHOLD = 50.0;

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
    private final CANrange ballSensor;
    private final DutyCycleOut dutyCycleControl;
    
    // State tracking
    private IntakeState currentState = IntakeState.IDLE;
    private boolean isBallHeld = false;
    private boolean wasLastStateBallPresent = false;
    private String errorMessage = "";

    public AlgaeIntake() {
        intakeMotor = new TalonFX(Constants.AlgaeIntakeConstants.algaeIntakeMotorID);
        ballSensor = new CANrange(Constants.AlgaeIntakeConstants.algaeIntakeCANrangeID);
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
        boolean isBallPresent = checkBallPresence();
        
        if (isBallPresent && !wasLastStateBallPresent) {
            isBallHeld = true;
            currentState = IntakeState.HOLDING;
            setIntakeSpeed(HOLD_SPEED);
        } else if (!isBallPresent && wasLastStateBallPresent && isBallHeld) {
            isBallHeld = false;
            currentState = IntakeState.IDLE;
        }
        
        wasLastStateBallPresent = isBallPresent;
    }
    
    private boolean checkBallPresence() {
        double distance = ballSensor.getDistance().getValueAsDouble();
        double ambient = ballSensor.getAmbientSignal().getValueAsDouble();
        
        return distance < BALL_DETECTION_THRESHOLD && ambient < AMBIENT_THRESHOLD;
    }
    
    public void intake() {
        if (!isBallHeld && currentState != IntakeState.ERROR) {
            setIntakeSpeed(INTAKE_SPEED);
            currentState = IntakeState.INTAKING;
        }
    }
    
    public void reverse() {
        if (currentState != IntakeState.ERROR) {
            setIntakeSpeed(REVERSE_SPEED);
            isBallHeld = false;
            currentState = IntakeState.REVERSING;
        }
    }
    
    public void stop() {
        if (currentState != IntakeState.ERROR) {
            setIntakeSpeed(0);
            currentState = IntakeState.IDLE;
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
        SmartDashboard.putNumber("Algae/Ball Distance", 
            ballSensor.getDistance().getValueAsDouble());
        SmartDashboard.putNumber("Algae/Ambient Noise", 
            ballSensor.getAmbientSignal().getValueAsDouble());
        
        if (currentState == IntakeState.ERROR) {
            SmartDashboard.putString("Algae/Error", errorMessage);
        }
    }

    public boolean isOperational() {
        return currentState != IntakeState.ERROR;
    }
}