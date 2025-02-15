// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Commands.ArmCommand;
import frc.robot.Commands.IntegratedMechanismCommand;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
     private final AlgaeIntake m_algaeIntake = new AlgaeIntake(); 
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
    private final CoralIntake m_coralIntake = new CoralIntake();
    // private final IntegratedMechanismSubsystem mechanism = 
    // new IntegratedMechanismSubsystem(m_elevatorSubsystem, m_ArmSubsystem);
    private final SafetySubsystem m_safetySystem = new SafetySubsystem(m_elevatorSubsystem, m_ArmSubsystem, 
    m_coralIntake, m_algaeIntake);


    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        //Named commands go below----------------

        //---------------------------------------

        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
     
        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));  
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
        
    ;
        
    m_safetySystem.setDefaultCommand(
    new StowOnIntakeCommand(m_safetySystem, m_algaeIntake, m_coralIntake)
);

// joystick.povUp().onTrue(new ArmElevatorToPositionCommand(m_safetySystem, 22.28, 65));    // L4
// joystick.povRight().onTrue(new ArmElevatorToPositionCommand(m_safetySystem, 16.5, 75));  // L3
// joystick.povDown().onTrue(new ArmElevatorToPositionCommand(m_safetySystem, 11.0, 90));   // L2
// joystick.povLeft().onTrue(new ArmElevatorToPositionCommand(m_safetySystem, 14.5, -125)); // PICKUP

joystick.povUp().onTrue(new ElevatorTest(m_elevatorSubsystem, Constants.SafetyConstants.L4[0]));    // L4
joystick.povRight().onTrue(new ElevatorTest(m_elevatorSubsystem, Constants.SafetyConstants.L3[0]));  // L3
joystick.povDown().onTrue(new ElevatorTest(m_elevatorSubsystem, Constants.SafetyConstants.L2[0]));   // L2
joystick.povLeft().onTrue(new ArmElevatorToPositionCommand(m_safetySystem, 14.5, -125)); // PICKUP

joystick.rightBumper().whileTrue(new ArmCommand(m_ArmSubsystem, Constants.SafetyConstants.L2[1]));
joystick.rightBumper().onFalse(new ArmCommand(m_ArmSubsystem, 0));



joystick.a().onTrue(new ArmElevatorToPositionCommand(m_safetySystem, 11.0, 0)); // stow




        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
