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
    private static final double PULLEY_RATIO = 5.0;
    private static final double INTAKE_SPEED = 0.7;
    private static final double REVERSE_SPEED = -1;
    private static final double HOLD_SPEED = 0.05;
    private static final double BALL_DETECTION_THRESHOLD = .2;
    private static final double AMBIENT_THRESHOLD = 50.0; // Lower is better
    
    private final TalonFX intakeMotor;
    private final CANrange ballSensor;
    private final DutyCycleOut dutyCycleControl;
    
    private boolean isBallHeld = false;
    private boolean wasLastStateBallPresent = false;
    private boolean isIntaking = false;
    private boolean isReversing = false;
    private String currentState = "Idle";
    
    public AlgaeIntake() {
        intakeMotor = new TalonFX(Constants.AlgaeIntakeConstants.algaeIntakeMotorID);
        ballSensor = new CANrange(Constants.AlgaeIntakeConstants.algaeIntakeCANrangeID);
        dutyCycleControl = new DutyCycleOut(0);
        
        configureMotor();
    }
    
    private void configureMotor() {
        var motorConfig = new TalonFXConfiguration();
        motorConfig.Feedback.SensorToMechanismRatio = PULLEY_RATIO;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        
        intakeMotor.getConfigurator().apply(motorConfig);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    
    @Override
    public void periodic() {
        updateBallDetectionState();
        updateDashboard();
        
        // Handle holding state in periodic
        if (isBallHeld && !isIntaking && !isReversing) {
            setIntakeSpeed(HOLD_SPEED);
            currentState = "Holding";
        }
    }
    
    private void updateDashboard() {
        SmartDashboard.putBoolean("Has Ball", isBallHeld);
        SmartDashboard.putBoolean("Is Intaking", isIntaking);
        SmartDashboard.putBoolean("Is Reversing", isReversing);
        SmartDashboard.putNumber("Ball Distance", ballSensor.getDistance().getValueAsDouble());
        SmartDashboard.putString("Current State", currentState);
        SmartDashboard.putNumber("Ambient Noise", ballSensor.getAmbientSignal().getValueAsDouble());
    }
    
    private void updateBallDetectionState() {
        boolean isBallPresent = checkBallPresence();
        
        if (isBallPresent) {
            if (!wasLastStateBallPresent) {
                isBallHeld = true;
                currentState = "Holding";
                setIntakeSpeed(HOLD_SPEED);
            }
        } else {
            if (wasLastStateBallPresent && isBallHeld) {
                isBallHeld = false;
                currentState = "Ball Lost";
            }
        }
        
        wasLastStateBallPresent = isBallPresent;
    }
    
    private boolean checkBallPresence() {
        return ballSensor.getDistance().getValueAsDouble() < BALL_DETECTION_THRESHOLD && 
               ballSensor.getAmbientSignal().getValueAsDouble() < AMBIENT_THRESHOLD;
    }
    
    public void intake() {
        if (!isBallHeld) {
            setIntakeSpeed(INTAKE_SPEED);
            isIntaking = true;
            isReversing = false;
            currentState = "Intaking";
        }
    }
    
    public void reverse() {
        setIntakeSpeed(REVERSE_SPEED);
        isBallHeld = false;
        isIntaking = false;
        isReversing = true;
        currentState = "Reversing";
    }
    
    public void stop() {
        setIntakeSpeed(0);
        isIntaking = false;
        isReversing = false;
        currentState = "Idle";
    }
    
    private void setIntakeSpeed(double speed) {
        intakeMotor.setControl(dutyCycleControl.withOutput(speed));
    }
    
    public boolean hasBall() {
        return isBallHeld;
    }
}