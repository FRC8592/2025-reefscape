package frc.robot.commands;

import frc.robot.commands.proxies.NewtonCommand;

public class OuttakeCommand  extends NewtonCommand{
    public OuttakeCommand(){
        super(
            intake.commands.outtakeCommand()
            
        );
    }
}
