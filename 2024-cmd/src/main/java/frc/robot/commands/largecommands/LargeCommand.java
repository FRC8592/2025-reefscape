package frc.robot.commands.largecommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.commands.NewtonCommands;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.scoring.Scoring;

public abstract class LargeCommand extends Command {
    protected static Swerve swerve;
    protected static Scoring scoring;
    protected static LEDs leds;
    
    public static void addSubsystems(Swerve swerve, Scoring scoring, LEDs leds){
        LargeCommand.swerve = swerve;
        LargeCommand.scoring = scoring;
        LargeCommand.leds = leds;
    }
    // Require at least one subsystem to be passed in
    public LargeCommand(Subsystem requirement1, Subsystem... moreRequirements){
        addRequirements(requirement1);
        addRequirements(moreRequirements);
    }
} 