package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
    private final TalonFX intakeMotor;
    
    private static final double INTAKE_SPEED = 0.7;
    private static final double HOLD_TORQUE = 0.2;
    private static final double CURRENT_THRESHOLD = 30.0;
    
    /* Uncomment when CANRange is added
    private final DigitalInput canRangeSensor;
    private static final int CAN_RANGE_DIO_PORT = 0;
    */
    
    public CoralIntake() {
        intakeMotor = new TalonFX(Constants.CoralIntakeConstants.coralIntakeMotorID);
        
        /* Uncomment when CANRange is added
        canRangeSensor = new DigitalInput(CAN_RANGE_DIO_PORT);
        */
        
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.Feedback.SensorToMechanismRatio = 1.0;
        configs.ClosedLoopGeneral.ContinuousWrap = false;
        configs.CurrentLimits.StatorCurrentLimit = 40;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        
        intakeMotor.getConfigurator().apply(configs);
    }
    
    public void intakeCoral() {
        intakeMotor.setControl(new DutyCycleOut(INTAKE_SPEED));
    }
    
    public void holdCoral() {
        intakeMotor.setControl(new DutyCycleOut(HOLD_TORQUE));
    }
    
    public void stop() {
        intakeMotor.setControl(new DutyCycleOut(0));
    }
    
    public boolean hasCoral() {
        double currentAmps = intakeMotor.getStatorCurrent().getValueAsDouble();
        return currentAmps > CURRENT_THRESHOLD;
        
        /* Uncomment when CANRange is added and remove current sensing
        return !canRangeSensor.get();
        */
    }
    
    @Override
    public void periodic() {
        if (hasCoral()) {
            holdCoral();
        }
    }
}