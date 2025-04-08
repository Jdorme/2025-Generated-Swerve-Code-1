package frc.robot.Commands.AutoAlign;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ReefAlignmentSubsystem;
import frc.robot.subsystems.ReefAlignmentSubsystem.ReefSide;

/**
 * Factory for creating reef alignment commands.
 */
public class ReefCommandFactory {
    private final ReefAlignmentSubsystem m_alignmentSubsystem;
    private final CommandSwerveDrivetrain m_drivetrain;
    
    /**
     * Creates a new ReefCommandFactory.
     * 
     * @param alignmentSubsystem The reef alignment subsystem
     * @param drivetrain The swerve drivetrain
     */
    public ReefCommandFactory(ReefAlignmentSubsystem alignmentSubsystem, CommandSwerveDrivetrain drivetrain) {
        m_alignmentSubsystem = alignmentSubsystem;
        m_drivetrain = drivetrain;
    }
    
    /**
     * Creates a command to align to the left pole of the reef.
     * Uses the ElevatorCam by default.
     * 
     * @return The command
     */
    public Command alignToLeftPole() {
        return new AlignToReefCommand(m_alignmentSubsystem, m_drivetrain, ReefSide.LEFT_POLE);
    }
    
    /**
     * Creates a command to align to the right pole of the reef.
     * Uses the EndGameCam by default.
     * 
     * @return The command
     */
    public Command alignToRightPole() {
        return new AlignToReefCommand(m_alignmentSubsystem, m_drivetrain, ReefSide.RIGHT_POLE);
    }
    
    /**
     * Creates a command to align to the left pole of the reef and then run another command.
     * 
     * @param thenCommand The command to run after alignment
     * @return The command sequence
     */
    public Command alignToLeftPoleThen(Command thenCommand) {
        return Commands.sequence(
            alignToLeftPole(),
            thenCommand
        );
    }
    
    /**
     * Creates a command to align to the right pole of the reef and then run another command.
     * 
     * @param thenCommand The command to run after alignment
     * @return The command sequence
     */
    public Command alignToRightPoleThen(Command thenCommand) {
        return Commands.sequence(
            alignToRightPole(),
            thenCommand
        );
    }
}