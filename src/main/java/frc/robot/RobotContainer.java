package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ArmConstants;
import frc.robot.Commands.ElevatorTest;
import frc.robot.Commands.ManualElevatorTest;
import frc.robot.Commands.ArmElevatorToPositionCommand;
import frc.robot.Commands.StowOnIntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntegratedMechanismSubsystem;
import frc.robot.subsystems.SafetySubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Commands.ArmCommand;
import frc.robot.Commands.IntegratedMechanismCommand;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private final AlgaeIntake m_algaeIntake = new AlgaeIntake(); 
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
    private final CoralIntake m_coralIntake = new CoralIntake();
    private final VisionSubsystem m_visionSubsystem;
    private final SafetySubsystem m_safetySystem = new SafetySubsystem(m_elevatorSubsystem, m_ArmSubsystem, 
    m_coralIntake, m_algaeIntake);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        m_visionSubsystem = new VisionSubsystem(drivetrain);
        
        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
        configureVisionTestBindings();
    }

    private void configureBindings() {
       // First, let's modify the default drive command to be more explicit about its behavior
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> {
            // Get our joystick inputs, but add deadbands to prevent drift
            double forwardInput = -joystick.getLeftY();
            double sidewaysInput = -joystick.getLeftX();
            double rotationInput = -joystick.getRightX();
            
            // Only apply rotation if we're actually trying to rotate
            // This helps prevent unwanted spinning
            if (Math.abs(rotationInput) < 0.1) {
                rotationInput = 0.0;
            }
            
            return drive.withVelocityX(forwardInput * MaxSpeed)
                       .withVelocityY(sidewaysInput * MaxSpeed)
                       .withRotationalRate(rotationInput * MaxAngularRate);
        })
    );
     
        m_algaeIntake.setDefaultCommand(
            new RunCommand(
                () -> {
                    if (!m_algaeIntake.hasBall()) {
                        m_algaeIntake.stop();
                    }
                },
                m_algaeIntake
            )
        );
  
        joystick.rightTrigger()
            .whileTrue(new RunCommand(
                () -> m_algaeIntake.intake(),
                m_algaeIntake
            ));
  
        joystick.leftTrigger()
            .whileTrue(new RunCommand(
                () -> m_algaeIntake.reverse(),
                m_algaeIntake
            ));
        
        m_safetySystem.setDefaultCommand(
            new StowOnIntakeCommand(m_safetySystem, m_algaeIntake, m_coralIntake)
        );

        joystick.povUp().onTrue(new ElevatorTest(m_elevatorSubsystem, Constants.SafetyConstants.L4[0]));
        joystick.povRight().onTrue(new ElevatorTest(m_elevatorSubsystem, Constants.SafetyConstants.L3[0]));
        joystick.povDown().onTrue(new ElevatorTest(m_elevatorSubsystem, Constants.SafetyConstants.L2[0]));
        joystick.povLeft().onTrue(new ArmElevatorToPositionCommand(m_safetySystem, 14.5, -125));

        joystick.rightBumper().whileTrue(new ArmCommand(m_ArmSubsystem, Constants.SafetyConstants.L2[1]));
        joystick.rightBumper().onFalse(new ArmCommand(m_ArmSubsystem, 0));

        joystick.a().onTrue(new ArmElevatorToPositionCommand(m_safetySystem, 11.0, 0));

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureVisionTestBindings() {
        if (RobotBase.isSimulation()) {
            joystick.b().onTrue(Commands.runOnce(() -> {
                Pose2d simulatedVisionMeasurement = new Pose2d(
                    3.0, 3.0, Rotation2d.fromDegrees(90)
                );
                m_visionSubsystem.processSimulatedMeasurement(simulatedVisionMeasurement);
            }));
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}