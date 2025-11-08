package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

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
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.Commands.AutoAlign.AlignToReefTagLeft;
import frc.robot.Commands.AutoAlign.AlignToReefTagRight;
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

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.65).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.03)  // Reduced from 0.1 to 0.03
    .withRotationalDeadband(MaxAngularRate * 0.03)  // Reduced from 0.1 to 0.03
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
    private final SafeInitializationCommand m_safetyInitCommand = new SafeInitializationCommand(
    m_safetySystem, m_ArmSubsystem, m_elevatorSubsystem);
    // Replace PhotonVision with Limelight
    
    private final SendableChooser<Command> autoChooser;
    
    public RobotContainer() {
        // Build the auto chooser
        registerAutonomousCommands();
        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);
        // Register named commands for PathPlanner BEFORE building the auto chooser
        
        
        

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
        drive.withVelocityX(applyJoystickCurve((-joystick.getLeftY()) * (MaxSpeed/3)))
             .withVelocityY(applyJoystickCurve((-joystick.getLeftX()) * (MaxSpeed/3)))
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
        joystick.leftTrigger().whileTrue(Commands.run(() -> m_algaeIntake.reverse()));

        m_endgameLift.setDefaultCommand(Commands.run(() -> m_endgameLift.stop(), m_endgameLift));
        joystick.rightBumper().whileTrue(Commands.run(() -> m_endgameLift.liftUp(), m_endgameLift));
        joystick.leftBumper().whileTrue(Commands.run(() -> m_endgameLift.liftDown(), m_endgameLift));

        // Using D-pad for scoring commands with direction constants
        joystick.povUp().onTrue(new L4ScoreCommand(m_safetySystem, m_coralIntake, m_elevatorSubsystem, m_ArmSubsystem));
        joystick.povLeft().onTrue(new L3ScoreCommand(m_safetySystem, m_coralIntake, m_elevatorSubsystem, m_ArmSubsystem));
        joystick.povDown().onTrue(new L2ScoreCommand(m_safetySystem, m_coralIntake, m_elevatorSubsystem, m_ArmSubsystem));
        joystick.povRight().onTrue(new ArmElevatorToPositionCommand(m_safetySystem, 4.0, 0));
        //joystick.x().onTrue(new AlignToReefTagRight(drivetrain));
        //joystick.a().onTrue(new AlignToReefTagLeft(drivetrain));
        // Y+B combination (should come first)
        joystick.y().and(joystick.b())
        .onTrue(new L1ScoreCommand(m_safetySystem, m_coralIntake, m_elevatorSubsystem, m_ArmSubsystem));

        // Y only when B is not pressed
       // joystick.y().and(joystick.b().negate())
        //.onTrue(new L3AlgaeCommand(m_safetySystem, m_algaeIntake));

        // B only when Y is not pressed
        //joystick.b().and(joystick.y().negate())
        //.onTrue(new AlgaeNetCommand(m_safetySystem, m_algaeIntake));
        
        joystick.back().onTrue(new ArmClimbPositionCommand(m_safetySystem, m_ArmSubsystem, m_elevatorSubsystem, m_algaeIntake));

        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        
        joystick.x().onTrue(new L2AlgaeCommand(m_safetySystem, m_algaeIntake));
        
        // Button to move to processor position
        //joystick.a().onTrue(new ProcessorCommand(m_safetySystem, m_algaeIntake));
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

    /**
     * Preload all autonomous routines to avoid delays during autonomous mode
     */
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}