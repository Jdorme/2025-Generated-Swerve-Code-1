package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralIntake extends SubsystemBase {
    // Physical constants
    private static final double PULLEY_RATIO = 5.0;
    private static final double CURRENT_LIMIT = 40.0;

    // Control constants
    private static final double INTAKE_SPEED = .25;
    private static final double HOLD_SPEED = 0;
    private static final double REVERSE_SPEED = -.5;

    // Sensor thresholds
    private static final double CORAL_DETECTION_THRESHOLD = .1;
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
    private final CANrange coralSensor;
    private final DutyCycleOut dutyCycleControl;
    
    // State tracking
    private IntakeState currentState = IntakeState.IDLE;
    private boolean isCoralHeld = false;
    private boolean wasLastStateCoralPresent = false;
    private String errorMessage = "";
    private boolean manualControl = false;  // New flag for manual control

    public CoralIntake() {
        intakeMotor = new TalonFX(Constants.CoralIntakeConstants.coralIntakeMotorID);
        coralSensor = new CANrange(Constants.CoralIntakeConstants.coralIntakeCANrangeID);
        dutyCycleControl = new DutyCycleOut(0);
        
        try {
            configureMotor();
        } catch (Exception e) {
            handleError("Motor configuration failed: " + e.getMessage());
        }
    }
    
    private void configureMotor() {
        var motorConfig = new TalonFXConfiguration();
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
                if (!manualControl) {  // Only do automatic control if not in manual mode
                    updateCoralDetectionState();
                }
                updateDashboard();
            } catch (Exception e) {
                handleError("Periodic update failed: " + e.getMessage());
            }
        }
    }

    private void updateCoralDetectionState() {
        boolean isCoralPresent = checkCoralPresence();
        
        if (isCoralPresent && !wasLastStateCoralPresent) {
            isCoralHeld = true;
            currentState = IntakeState.HOLDING;
            setIntakeSpeed(HOLD_SPEED);
        } else if (!isCoralPresent && wasLastStateCoralPresent && isCoralHeld && !manualControl) {
            isCoralHeld = false;
            currentState = IntakeState.IDLE;
        }
        
        wasLastStateCoralPresent = isCoralPresent;
    }
    
    private boolean checkCoralPresence() {
        double distance = coralSensor.getDistance().getValueAsDouble();
        double ambient = coralSensor.getAmbientSignal().getValueAsDouble();
        return distance < CORAL_DETECTION_THRESHOLD && ambient < AMBIENT_THRESHOLD;
    }
    
    public void intakeCoral() {
        if (!isCoralHeld && currentState != IntakeState.ERROR) {
            manualControl = false;  // Return to automatic control
            setIntakeSpeed(INTAKE_SPEED);
            currentState = IntakeState.INTAKING;
        }
    }
    
    public void reverse() {
        if (currentState != IntakeState.ERROR) {
            manualControl = true;  // Enable manual control
            setIntakeSpeed(REVERSE_SPEED);
            currentState = IntakeState.REVERSING;
        }
    }
    
    public void stop() {
        if (currentState != IntakeState.ERROR) {
            manualControl = false;  // Return to automatic control
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
            System.err.println("Failed to stop motor in error handler: " + e.getMessage());
        }
    }
    
    public boolean hasCoral() {
        return isCoralHeld;
    }
    
    private void updateDashboard() {
        SmartDashboard.putBoolean("Coral/Has Coral", isCoralHeld);
        SmartDashboard.putString("Coral/State", currentState.toString());
        SmartDashboard.putNumber("Coral/Coral Distance", 
            coralSensor.getDistance().getValueAsDouble());
        SmartDashboard.putNumber("Coral/Ambient Noise", 
            coralSensor.getAmbientSignal().getValueAsDouble());
        SmartDashboard.putBoolean("Coral/Manual Control", manualControl);
        
        if (currentState == IntakeState.ERROR) {
            SmartDashboard.putString("Coral/Error", errorMessage);
        }
    }

    public boolean isOperational() {
        return currentState != IntakeState.ERROR;
    }
}