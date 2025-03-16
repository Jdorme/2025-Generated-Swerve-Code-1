package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
import frc.robot.Constants.SafetyConstants;
import frc.robot.Commands.ElevatorTest;
import frc.robot.Commands.StowOnIntakeCommand;
import frc.robot.Commands.AlgaeCommands.FloorIntakePositionCommand;
import frc.robot.Commands.AlgaeCommands.L2AlgaeCommand;
import frc.robot.Commands.AlgaeCommands.L3AlgaeCommand;
import frc.robot.Commands.AlgaeCommands.ProcessorCommand;
import frc.robot.Commands.CoralCommands.CoralIntakeL2AlgaeCommand;
import frc.robot.Commands.CoralCommands.L2ScoreCommand;
import frc.robot.Commands.CoralCommands.L3ScoreCommand;
import frc.robot.Commands.CoralCommands.L4ScoreCommand;
import frc.robot.Commands.ArmElevatorToPositionCommand;
import frc.robot.Commands.AutoCommandFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndgameLiftSubsystem;
import frc.robot.subsystems.SafetySubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Commands.ArmClimbPositionCommand;
import frc.robot.Commands.ArmCommand;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.65).in(RadiansPerSecond);

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
    private final SafetySubsystem m_safetySystem = new SafetySubsystem(m_elevatorSubsystem, m_ArmSubsystem, 
    m_coralIntake, m_algaeIntake);
    private final EndgameLiftSubsystem m_endgameLift = new EndgameLiftSubsystem();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        
        // Register named commands for PathPlanner BEFORE building the auto chooser
        registerAutonomousCommands();
        
        // Build the auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }
    
    /**
     * Register commands that can be used in autonomous routines.
     * This method registers all our commands with PathPlanner's named command system.
     */
    private void registerAutonomousCommands() {
        // Register commands using the AutoCommandFactory
        AutoCommandFactory.registerNamedCommands(
            m_safetySystem,
            m_algaeIntake,
            m_coralIntake,
            m_elevatorSubsystem,
            m_ArmSubsystem
        );
        
        // Add any additional autonomous-specific commands here if needed
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * (MaxSpeed/1.5)) //Divide by 4 to reduce max speed
                    .withVelocityY(-joystick.getLeftX() * (MaxSpeed/1.5))
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

        m_safetySystem.setDefaultCommand(
            new StowOnIntakeCommand(m_safetySystem, m_algaeIntake, m_coralIntake)
        );

        joystick.rightTrigger().whileTrue(new CoralIntakeL2AlgaeCommand(m_coralIntake, m_safetySystem));
        //joystick.rightTrigger().whileTrue(Commands.run(() -> m_algaeIntake.intake()));
        joystick.leftTrigger().whileTrue(Commands.run(() -> m_algaeIntake.reverse()));

//        joystick.x().whileTrue(new L2AlgaeCommand(m_algaeIntake, m_elevatorSubsystem,m_ArmSubsystem));

        m_endgameLift.setDefaultCommand(Commands.run(() -> m_endgameLift.stop(), m_endgameLift));
        joystick.rightBumper().whileTrue(Commands.run(() -> m_endgameLift.liftUp(), m_endgameLift));
        joystick.leftBumper().whileTrue(Commands.run(() -> m_endgameLift.liftDown(), m_endgameLift));

        // Using D-pad for scoring commands with direction constants
        joystick.povUp().onTrue(new L4ScoreCommand(m_safetySystem, m_coralIntake, m_elevatorSubsystem, m_ArmSubsystem));
        joystick.povLeft().onTrue(new L3ScoreCommand(m_safetySystem, m_coralIntake, m_elevatorSubsystem, m_ArmSubsystem));
        joystick.povDown().onTrue(new L2ScoreCommand(m_safetySystem, m_coralIntake, m_elevatorSubsystem, m_ArmSubsystem));
        joystick.povRight().onTrue(new ArmElevatorToPositionCommand(m_safetySystem, 14.0, 0));
        
        joystick.back().onTrue(new ArmClimbPositionCommand(m_safetySystem, m_ArmSubsystem, m_elevatorSubsystem, m_algaeIntake));


        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(jo\ystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.x().onTrue(new L2AlgaeCommand(m_safetySystem, m_algaeIntake));
        joystick.y().onTrue(new L3AlgaeCommand(m_safetySystem, m_algaeIntake));
        
        // Button to move to processor position
        joystick.a().onTrue(new ProcessorCommand(m_safetySystem, m_algaeIntake));

        
        // Button to move to net position
        // joystick.b().onTrue(new ArmElevatorToPositionCommand(m_safetySystem, 
        //     SafetyConstants.NET_ALGAE[0], 
        //     SafetyConstants.NET_ALGAE[1]));

       // joystick.start().onTrue(new FloorIntakePositionCommand(m_safetySystem, m_ArmSubsystem, m_elevatorSubsystem, m_algaeIntake));


        // Add Reef Alignment Bindings
        // Align to the left side of the Reef AprilTag
        // joystick.x().onTrue(
        //     Commands.runOnce(() -> 
        //         new ReefAlignmentCommand(
        //             drivetrain, 
        //             m_visionSubsystem, 
        //             ReefAlignmentCommand.AlignmentSide.LEFT
        //         ).schedule()
        //     )
        // );

        // // Align to the right side of the Reef AprilTag
        // joystick.b().onTrue(
        //     Commands.runOnce(() -> 
        //         new ReefAlignmentCommand(
        //             drivetrain, 
        //             m_visionSubsystem, 
        //             ReefAlignmentCommand.AlignmentSide.RIGHT
        //         ).schedule()
        //     )
        // );
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}