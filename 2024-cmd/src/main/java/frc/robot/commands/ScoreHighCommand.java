package frc.robot.commands;

import frc.robot.commands.proxies.NewtonCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot.Positions;

public class ScoreHighCommand extends NewtonCommand {
    public ScoreHighCommand(Intake intake){
        super(
            pivot.commands.setPivotPositionCommand(Positions.SCORE_HIGH).andThen(
                intake.intakeCommand(),intake.stopCommand()            )
            
        );
    }
}
