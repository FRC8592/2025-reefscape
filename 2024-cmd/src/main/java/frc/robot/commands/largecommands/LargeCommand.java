package frc.robot.commands.largecommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;

public abstract class LargeCommand extends Command {
    protected static Swerve swerve;
    protected static Vision vision;
    public static void addSubsystems(Swerve swerve, Vision vision){
        LargeCommand.swerve = swerve;
        LargeCommand.vision = vision;
    }
    // Require at least one subsystem to be passed in
    public LargeCommand(Subsystem requirement1, Subsystem... moreRequirements){
        addRequirements(requirement1);
        addRequirements(moreRequirements);
    }
}