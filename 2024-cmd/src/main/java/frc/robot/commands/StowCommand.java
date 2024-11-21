package frc.robot.commands;

import frc.robot.commands.proxies.NewtonCommand;

public class StowCommand extends NewtonCommand{
    public StowCommand(){
        super(
            pivot.commands.setPivotPositionCommand(Positions.REST).alongWith(
                stopSubsystems(intake.commands)
            )
        );
    }
}
