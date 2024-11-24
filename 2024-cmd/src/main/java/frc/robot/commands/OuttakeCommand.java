package frc.robot.commands;

import frc.robot.commands.proxies.NewtonCommand;
import frc.robot.subsystems.intake.Intake;

public class OuttakeCommand  extends NewtonCommand{
    public OuttakeCommand(Intake intake){
        super(
            intake.outtakeCommand()
            
        );
    }
}
