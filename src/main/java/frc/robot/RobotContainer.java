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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SafetyConstants;
import frc.robot.Commands.ElevatorTest;
import frc.robot.Commands.StowOnIntakeCommand;
import frc.robot.Commands.AlgaeCommands.AlgaeNetCommand;
import frc.robot.Commands.AlgaeCommands.FloorIntakePositionCommand;
import frc.robot.Commands.AlgaeCommands.L2AlgaeCommand;
import frc.robot.Commands.AlgaeCommands.L3AlgaeCommand;
import frc.robot.Commands.AlgaeCommands.ProcessorCommand;
import frc.robot.Commands.AutoAlign.AprilTagPathCommand;
import frc.robot.Commands.AutoAlign.ReefAlignmentCommand;
import frc.robot.Commands.CoralCommands.CoralIntakeL2AlgaeCommand;
import frc.robot.Commands.CoralCommands.L1ScoreCommand;
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
import frc.robot.Commands.AlgaeProcessorScoreCommand;
import frc.robot.Commands.ArmClimbPositionCommand;
import frc.robot.Commands.ArmCommand;
import frc.robot.Commands.SafeInitializationCommand;
import frc.robot.subsystems.PhotonVisionSubsystem;




public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.65).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.03)  // Reduced from 0.1 to 0.03
    .withRotationalDeadband(MaxAngularRate * 0.03)  // Reduced from 0.1 to 0.03
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
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
    private final SafeInitializationCommand m_safetyInitCommand = new SafeInitializationCommand(
    m_safetySystem, m_ArmSubsystem, m_elevatorSubsystem);
    private final PhotonVisionSubsystem m_photonVision = new PhotonVisionSubsystem();
    private final ReefAlignmentCommand m_ReefAlignmentCommand = new ReefAlignmentCommand(drivetrain, m_photonVision, null, drive);
    
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
    public void runSafetyInitialization() {
    // Cancel any running commands that might interfere
    CommandScheduler.getInstance().cancelAll();
    
    // Schedule the safety initialization command
    m_safetyInitCommand.schedule();
    
    // Note: We don't wait for it to complete here, as that would block the robot thread
    // The command will run in the background, preventing other commands from
    // taking control of the subsystems until it's finished
    
    System.out.println("Safety initialization sequence started");
}

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
        drive.withVelocityX(applyJoystickCurve(-joystick.getLeftY()) * (MaxSpeed/1.875))
             .withVelocityY(applyJoystickCurve(-joystick.getLeftX()) * (MaxSpeed/1.875))
             .withRotationalRate(applyJoystickCurve(-joystick.getRightX()) * MaxAngularRate*1.125)
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
        joystick.povRight().onTrue(new ArmElevatorToPositionCommand(m_safetySystem, 4.0, 0));
        //joystick.y().and(joystick.b()).onTrue(new L1ScoreCommand(m_safetySystem, m_coralIntake, m_elevatorSubsystem, m_ArmSubsystem));
        
        joystick.back().onTrue(new ArmClimbPositionCommand(m_safetySystem, m_ArmSubsystem, m_elevatorSubsystem, m_algaeIntake));


        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(jo\ystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.x().onTrue(new L2AlgaeCommand(m_safetySystem, m_algaeIntake));
        joystick.y().onTrue(new L3AlgaeCommand(m_safetySystem, m_algaeIntake));
        // Add Reef Alignment Bindings
        // Align to the left side of the Reef AprilTag
        //joystick.a().whileTrue(new ReefAlignmentCommand(drivetrain, m_photonVision, ReefAlignmentCommand.AlignmentSide.LEFT,drive));
        // joystick.a().onTrue(new AprilTagPathCommand( m_photonVision, 
        // drivetrain, 
        // .22, // Y-offset: 1 meter to the left of the tag 
        // 2.5, // Max velocity: 3 m/s 
        // 1.5, // Max acceleration: 2 m/s² 
        // 6,7,8,9,10,11,17,18,19,20,21,22 ));// List of all allowed tag IDs ));
        // joystick.b().onTrue(new AprilTagPathCommand( m_photonVision, 
        // drivetrain, 
        // -.22, // Y-offset: 1 meter to the left of the tag 
        // 2.5, // Max velocity: 3 m/s 
        // 1.5, // Max acceleration: 2 m/s² 
        // 6,7,8,9,10,11,17,18,19,20,21,22));
        //Align to the right side of the Reef AprilTag
        //joystick.b().whileTrue(new ReefAlignmentCommand(drivetrain, m_photonVision, ReefAlignmentCommand.AlignmentSide.RIGHT,drive));

        // Add Reef Alignment Bindings
        // Align to the left side of the Reef AprilTag
        
        // Basic auto-align to nearest AprilTag
        
        // Button to move to processor position
        joystick.a().onTrue(new ProcessorCommand(m_safetySystem, m_algaeIntake));

        
        // Button to move to net position
       joystick.b().onTrue(new AlgaeNetCommand(m_safetySystem, m_algaeIntake));

        //  joystick.b().onTrue(Commands.runOnce(() -> m_elevatorSubsystem.setHeight(SafetyConstants.NET_ALGAE[0]), m_elevatorSubsystem));
        //  joystick.b().onTrue(Commands.runOnce(() -> m_ArmSubsystem.setAngle(SafetyConstants.NET_ALGAE[1]), m_ArmSubsystem));
        
            


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

    private double applyJoystickCurve(double input) {
        // Preserve the sign of the input
        double sign = Math.signum(input);
        // Take the absolute value to work with positive numbers
        double absInput = Math.abs(input);
        
        // Apply a power curve (squared) for reduced sensitivity at lower inputs
        // This makes the joystick less sensitive near center, allowing for finer control at low speeds
        double adjustedValue = Math.pow(absInput, 2);
        
        // Return the adjusted value with the original sign
        return sign * adjustedValue;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}