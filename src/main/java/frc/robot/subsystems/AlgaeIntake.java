package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
    private static final double INTAKE_SPEED = 1;  // Using new, lower speed
    private static final double REVERSE_SPEED = -1.0;
    private static final double HOLD_SPEED = 0.05;
    

    // Sensor thresholds - ADJUSTABLE for testing
    private static final double BALL_DETECTION_THRESHOLD = 0.0625; // Increased from 0.05 to 0.5
    private static final double AMBIENT_THRESHOLD = 25;      // Increased from 50.0 to 100.0

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
    private boolean newHasBall;
    private boolean wasLastStateBallPresent = false;
    private String errorMessage = "";
    
    // Diagnostic data
    private double minDistance = Double.MAX_VALUE;
    private double maxDistance = Double.MIN_VALUE;
    private double minAmbient = Double.MAX_VALUE;
    private double maxAmbient = Double.MIN_VALUE;
    private int totalReadings = 0;
    private int positiveReadings = 0;
    private int cycleCounter = 0;
    

    public AlgaeIntake() {
        intakeMotor = new TalonFX(Constants.AlgaeIntakeConstants.algaeIntakeMotorID);
        ballSensor = new CANrange(Constants.AlgaeIntakeConstants.algaeIntakeCANrangeID);
        dutyCycleControl = new DutyCycleOut(0);
        double Current = intakeMotor.getStatorCurrent().getValueAsDouble();
        SmartDashboard.putNumber("TCurrent",Current);
        newHasBall = Current > -25; 
        try {
            configureMotor();
            System.out.println("Started AlgaeIntake diagnostics - CANrange Sensor ID: " + ballSensor.getDeviceID());
        } catch (Exception e) {
            handleError("Configuration failed: " + e.getMessage());
            e.printStackTrace();
        }
    }
    
    private void configureMotor() {
        var motorConfig = new TalonFXConfiguration();
        
        // Motor configuration
        motorConfig.Feedback.SensorToMechanismRatio = PULLEY_RATIO;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        intakeMotor.getConfigurator().apply(motorConfig);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    
    @Override
    public void periodic() {
        try {
            // Comment out or modify functions that use the CANrange
            // updateBallDetectionState(); 
            updateMotorState();
            // updateDiagnostics();
            
            // Update dashboard with minimal info that doesn't use CANrange
            SmartDashboard.putString("Algae/State", currentState.toString());
        } catch (Exception e) {
            // Just print the error but don't change state
            System.out.println("Ignored error: " + e.getMessage());
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
            System.out.println("BALL DETECTED! Distance: " + 
                               ballSensor.getDistance().getValueAsDouble() + 
                               ", Ambient: " + 
                               ballSensor.getAmbientSignal().getValueAsDouble());
        } else if (!isBallPresent && wasLastStateBallPresent && isBallHeld) {
            isBallHeld = false;
            currentState = IntakeState.IDLE;
            setIntakeSpeed(0);
            System.out.println("BALL LOST! Distance: " + 
                               ballSensor.getDistance().getValueAsDouble() + 
                               ", Ambient: " + 
                               ballSensor.getAmbientSignal().getValueAsDouble());
        }
        
        wasLastStateBallPresent = isBallPresent;
    }
    
    private boolean checkBallPresence() {
        double distance = ballSensor.getDistance().getValueAsDouble();
        double ambient = ballSensor.getAmbientSignal().getValueAsDouble();
        
        // Update statistics if we have valid readings
        if (!Double.isNaN(distance) && !Double.isInfinite(distance) && distance > 0) {
            minDistance = Math.min(minDistance, distance);
            maxDistance = Math.max(maxDistance, distance);
        }
        
        if (!Double.isNaN(ambient) && !Double.isInfinite(ambient) && ambient >= 0) {
            minAmbient = Math.min(minAmbient, ambient);
            maxAmbient = Math.max(maxAmbient, ambient);
        }
        
        boolean isDistanceInRange = distance < BALL_DETECTION_THRESHOLD && distance > 0;
        boolean isAmbientInRange = ambient < AMBIENT_THRESHOLD;
        boolean isPresent = isDistanceInRange && isAmbientInRange;
        
        // Update diagnostic counters
        totalReadings++;
        if (isPresent) {
            positiveReadings++;
        }
        
        return isPresent;
    }
    
    private void updateDiagnostics() {
        cycleCounter++;
        
        // Print detailed sensor data every 100 cycles
        if (cycleCounter % 100 == 0) {
            double distance = ballSensor.getDistance().getValueAsDouble();
            double ambient = ballSensor.getAmbientSignal().getValueAsDouble();
            
            System.out.println("===== CANrange Sensor Diagnostics (Cycle " + cycleCounter + ") =====");
            System.out.println("Current Distance: " + distance);
            System.out.println("Current Ambient: " + ambient);
            System.out.println("Min/Max Distance: " + minDistance + " / " + maxDistance);
            System.out.println("Min/Max Ambient: " + minAmbient + " / " + maxAmbient);
            System.out.println("Detection Rate: " + (positiveReadings * 100.0 / totalReadings) + "%");
            System.out.println("Detection Thresholds: Distance < " + BALL_DETECTION_THRESHOLD + 
                              ", Ambient < " + AMBIENT_THRESHOLD);
            
            // Check for potential issues
            if (distance <= 0) {
                System.out.println("WARNING: Sensor reporting zero or negative distance");
            }
            if (Double.isNaN(distance) || Double.isInfinite(distance)) {
                System.out.println("WARNING: Sensor reporting NaN or Infinite distance");
            }
            
            System.out.println("==================================================");
        }
    }
    
    public void intake() {
        if (!isBallHeld && currentState != IntakeState.ERROR) {
            setIntakeSpeed(INTAKE_SPEED);
            currentState = IntakeState.INTAKING;
            System.out.println("Starting intake at speed: " + INTAKE_SPEED);
        }
    }
    
    public void reverse() {
        if (currentState != IntakeState.ERROR) {
            setReverseIntakeSpeed(REVERSE_SPEED);
            currentState = IntakeState.REVERSING;
            System.out.println("Reversing intake at speed: " + REVERSE_SPEED);
        }
    }
    
    public void stop() {
        if (currentState != IntakeState.ERROR) {
            setIntakeSpeed(0);
            currentState = IntakeState.IDLE;
          //  System.out.println("Stopping intake");
        }
    }
    
    private void setIntakeSpeed(double speed) {
        try {
            if (newHasBall){intakeMotor.setControl(dutyCycleControl.withOutput(.15));
            }else{intakeMotor.setControl(dutyCycleControl.withOutput(.3));}
        } catch (Exception e) {
            handleError("Failed to set motor speed: " + e.getMessage());
            e.printStackTrace();
        }
    }
    private void setReverseIntakeSpeed(double speed) {
        try {
            intakeMotor.setControl(dutyCycleControl.withOutput(REVERSE_SPEED));
            
        } catch (Exception e) {
            handleError("Failed to set motor speed: " + e.getMessage());
            e.printStackTrace();
        }
    }
    private void handleError(String message) {
        // currentState = IntakeState.ERROR;
        // errorMessage = message;
        // try {
        //     intakeMotor.stopMotor();
        // } catch (Exception e) {
        //     System.err.println("Failed to stop motor in error handler: " + e.getMessage());
        //     e.printStackTrace();
        // }
    }
    
    public boolean hasBall() {
        return isBallHeld;
    }
    
    private void updateDashboard() {
        // Basic state info
        SmartDashboard.putBoolean("Algae/Has Ball", isBallHeld);
        SmartDashboard.putString("Algae/State", currentState.toString());
        
        // Current sensor readings
        double distance = ballSensor.getDistance().getValueAsDouble();
        double ambient = ballSensor.getAmbientSignal().getValueAsDouble();
        SmartDashboard.putNumber("Algae/Ball Distance", distance);
        SmartDashboard.putNumber("Algae/Ambient Light", ambient);
        
        // Detection criteria
        boolean isDistanceInRange = distance < BALL_DETECTION_THRESHOLD && distance > 0;
        boolean isAmbientInRange = ambient < AMBIENT_THRESHOLD;
        SmartDashboard.putBoolean("Algae/Distance Check", isDistanceInRange);
        SmartDashboard.putBoolean("Algae/Ambient Check", isAmbientInRange);
        SmartDashboard.putBoolean("Algae/Current Reading", isDistanceInRange && isAmbientInRange);
        
        // Range statistics
        if (minDistance != Double.MAX_VALUE) {
            SmartDashboard.putNumber("Algae/Min Distance", minDistance);
        }
        if (maxDistance != Double.MIN_VALUE) {
            SmartDashboard.putNumber("Algae/Max Distance", maxDistance);
        }
        if (minAmbient != Double.MAX_VALUE) {
            SmartDashboard.putNumber("Algae/Min Ambient", minAmbient);
        }
        if (maxAmbient != Double.MIN_VALUE) {
            SmartDashboard.putNumber("Algae/Max Ambient", maxAmbient);
        }
        
        SmartDashboard.putNumber("Algae/Detection Rate", 
                                 totalReadings > 0 ? (positiveReadings * 100.0 / totalReadings) : 0);
        
        if (currentState == IntakeState.ERROR) {
            SmartDashboard.putString("Algae/Error", errorMessage);
        }
    }

    public boolean isOperational() {
        return currentState != IntakeState.ERROR;
    }
}