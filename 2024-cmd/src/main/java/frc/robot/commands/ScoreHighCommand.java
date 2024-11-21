package frc.robot.commands;

import frc.robot.commands.proxies.NewtonCommand;

public class ScoreHighCommand extends NewtonCommand {
    public ScoreHighCommand(){
        super(
            pivot.commands.setPivotPositionCommand(Positions.SCORE_HIGH).andThen(
                intake.commands.scoreCommand(), stopSubsystems(intake.commands)
            )
            
        );
    }
}
