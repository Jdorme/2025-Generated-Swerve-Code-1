package frc.robot.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SafetyConstants;
import frc.robot.Commands.ArmElevatorToPositionCommand;
import frc.robot.Commands.AlgaeCommands.L2AlgaeCommand;
import frc.robot.Commands.AlgaeCommands.L3AlgaeCommand;
import frc.robot.Commands.CoralCommands.CoralIntakeL2AlgaeCommand;
import frc.robot.Commands.CoralCommands.L2ScoreCommand;
import frc.robot.Commands.CoralCommands.L3ScoreCommand;
import frc.robot.Commands.CoralCommands.L4ScoreCommand;
import frc.robot.Commands.CoralCommands.AutoCoralIntakeCommand;
import frc.robot.Commands.CoralCommands.AutoElevatorArmL3Command;
import frc.robot.Commands.CoralCommands.AutoElevatorArmL4Command;
import frc.robot.Commands.CoralCommands.AutoL3ScoreCommand;
import frc.robot.Commands.CoralCommands.AutoL4ScoreCommand;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SafetySubsystem;

/**
 * This class registers commands for use with PathPlanner's auto builder.
 * Each method here corresponds to a command that can be called by name in the PathPlanner interface.
 */
public class AutoCommandFactory {
    
    /**
     * Register all autonomous commands with PathPlanner's NamedCommands system
     */
    public static void registerNamedCommands(
            SafetySubsystem safetySystem,
            AlgaeIntake algaeIntake,
            CoralIntake coralIntake,
            ElevatorSubsystem elevator,
            ArmSubsystem arm) {
        
        // Register scoring commands
        NamedCommands.registerCommand("Score L2", new L2ScoreCommand(safetySystem, coralIntake, elevator, arm));
        NamedCommands.registerCommand("Score L3", new L3ScoreCommand(safetySystem, coralIntake, elevator, arm));
        NamedCommands.registerCommand("Score L4", new L4ScoreCommand(safetySystem, coralIntake, elevator, arm));
        NamedCommands.registerCommand("AutoL4ScoreCommand", new AutoL4ScoreCommand(safetySystem, coralIntake, elevator, arm));
        NamedCommands.registerCommand("AutoL3ScoreCommand", new AutoL3ScoreCommand(safetySystem, coralIntake, elevator, arm));

        

        // Register intake commands
        NamedCommands.registerCommand("Intake Algae L2", new L2AlgaeCommand(safetySystem, algaeIntake));
        NamedCommands.registerCommand("Intake Algae L3", new L3AlgaeCommand(safetySystem, algaeIntake));
        NamedCommands.registerCommand("Intake Coral", new CoralIntakeL2AlgaeCommand(coralIntake, safetySystem));
        
        // Register position commands
        NamedCommands.registerCommand("Stow", 
            new ArmElevatorToPositionCommand(safetySystem, SafetyConstants.STOWED[0], SafetyConstants.STOWED[1]));
        NamedCommands.registerCommand("Ground Algae Position", 
            new ArmElevatorToPositionCommand(safetySystem, SafetyConstants.GROUND_ALGAE[0], SafetyConstants.GROUND_ALGAE[1]));
        NamedCommands.registerCommand("L4Elevator", new AutoElevatorArmL4Command(elevator, arm, coralIntake));
        NamedCommands.registerCommand("L3Elevator", new AutoElevatorArmL3Command(elevator, arm));

        
        // Register intake actions
        NamedCommands.registerCommand("Algae Intake", Commands.run(() -> algaeIntake.intake(), algaeIntake)
            .withTimeout(3.0)); // Timeout to prevent running indefinitely
        NamedCommands.registerCommand("Algae Stop", Commands.runOnce(() -> algaeIntake.stop(), algaeIntake));
        NamedCommands.registerCommand("Algae Reverse", Commands.run(() -> algaeIntake.reverse(), algaeIntake)
            .withTimeout(1.0));
            
        NamedCommands.registerCommand("Coral Intake", Commands.run(() -> coralIntake.intakeCoral(), coralIntake)
            .withTimeout(3.0));
        NamedCommands.registerCommand("AutoCoralIntakeCommand", new AutoCoralIntakeCommand(coralIntake, safetySystem));
        NamedCommands.registerCommand("Coral Stop", Commands.runOnce(() -> coralIntake.stop(), coralIntake));
        NamedCommands.registerCommand("Coral Reverse", Commands.run(() -> coralIntake.reverse(), coralIntake)
            .withTimeout(1.0));

        
    }
}