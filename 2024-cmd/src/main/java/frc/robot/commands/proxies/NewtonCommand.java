package frc.robot.commands.proxies;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.SubsystemCommands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.swerve.Swerve;

public abstract class NewtonCommand extends WrapperCommand {
    protected static Pivot pivot = Pivot.getInstance();
    protected static Swerve swerve = Swerve.getInstance();
    //TODO: Add more subsystems here

    public NewtonCommand(Command command){
        super(command);
        setName(getClass().getSimpleName());
    }

    public static Command stopSubsystems(SubsystemCommands... toStop){
        Command result = Commands.none();
        for(SubsystemCommands s:toStop){
            result = result.alongWith(s.stopCommand());
        }
        return result;
    }
}
