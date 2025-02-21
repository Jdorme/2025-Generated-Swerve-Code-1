package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.EndgameLiftConstants;

public class EndgameLiftSubsystem extends SubsystemBase {
    // Motors
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;
    
    // Control request
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    
    public EndgameLiftSubsystem() {
        topMotor = new TalonFX(EndgameLiftConstants.liftMotorTopID);
        bottomMotor = new TalonFX(EndgameLiftConstants.liftMotorBottomID);
        
        configureMotors();
    }
    
    private void configureMotors() {
        var motorConfig = new TalonFXConfiguration();
        
        // Basic motor configuration
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Current limits for safety
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = EndgameLiftConstants.CURRENT_LIMIT;
        
        // Configure top motor
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        topMotor.getConfigurator().apply(motorConfig);
        
        // Configure bottom motor (inverted)
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        bottomMotor.getConfigurator().apply(motorConfig);
    }
    
    public void liftUp() {
        setSpeed(EndgameLiftConstants.LIFT_UP_SPEED);
    }
    
    public void liftDown() {
        setSpeed(EndgameLiftConstants.LIFT_DOWN_SPEED);
    }
    
    public void stop() {
        setSpeed(0);
    }
    
    private void setSpeed(double speed) {
        topMotor.setControl(dutyCycleControl.withOutput(speed));
        bottomMotor.setControl(dutyCycleControl.withOutput(speed));
    }
    
    @Override
    public void periodic() {
        // Update dashboard with motor currents for monitoring
        SmartDashboard.putNumber("EndgameLift/Top Current", 
            topMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("EndgameLift/Bottom Current", 
            bottomMotor.getSupplyCurrent().getValueAsDouble());
    }
}