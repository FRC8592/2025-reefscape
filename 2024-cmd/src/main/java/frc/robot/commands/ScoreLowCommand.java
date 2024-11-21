package frc.robot.commands;

import frc.robot.commands.proxies.NewtonCommand;

public class ScoreLowCommand extends NewtonCommand{
    public ScoreLowCommand(){
        super(
            pivot.commands.setPivotPositionCommand(Positions.GROUND).andThen(
                intake.commands.scoreCommand(), stopSubsystems(intake.commands)
            )
        
            
        );
    }
}
