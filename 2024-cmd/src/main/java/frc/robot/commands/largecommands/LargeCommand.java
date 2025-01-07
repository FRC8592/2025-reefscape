package frc.robot.commands.largecommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.Swerve;

public abstract class LargeCommand extends Command {
    protected static Swerve swerve;
    protected static Intake intake;
    protected static Pivot pivot;
    public static void addSubsystems(Swerve swerve, Intake intake, Pivot pivot){
        LargeCommand.swerve = swerve;
        LargeCommand.intake = intake;
        LargeCommand.pivot = pivot;
    }
    // Require at least one subsystem to be passed in
    public LargeCommand(Subsystem requirement1, Subsystem... moreRequirements){
        addRequirements(requirement1);
        addRequirements(moreRequirements);
    }
}