package frc.robot.commands.largecommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.scoring.Scoring;

public abstract class LargeCommand extends Command {
    protected static Swerve swerve;
    protected static Scoring scoring;
    public static void addSubsystems(Swerve swerve, Scoring scoring){
        LargeCommand.swerve = swerve;
        LargeCommand.scoring = scoring;
    }
    // Require at least one subsystem to be passed in
    public LargeCommand(Subsystem requirement1, Subsystem... moreRequirements){
        addRequirements(requirement1);
        addRequirements(moreRequirements);
    }
} 