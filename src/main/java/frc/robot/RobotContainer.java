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
import frc.robot.Commands.ReefAlignmentCommand;
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
        
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
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
     // Add Reef Alignment Bindings
    // Align to the left side of the Reef AprilTag
    joystick.x().onTrue(
        Commands.runOnce(() -> 
            new ReefAlignmentCommand(
                drivetrain, 
                m_visionSubsystem, 
                ReefAlignmentCommand.AlignmentSide.LEFT
            ).schedule()
        )
    );

    // Align to the right side of the Reef AprilTag
    joystick.b().onTrue(
        Commands.runOnce(() -> 
            new ReefAlignmentCommand(
                drivetrain, 
                m_visionSubsystem, 
                ReefAlignmentCommand.AlignmentSide.RIGHT
            ).schedule()
        )
    );
    }



    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}