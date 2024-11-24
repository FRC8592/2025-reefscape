package frc.robot.commands;

import frc.robot.commands.proxies.NewtonCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot.Positions;

public class HumanPlayerIntakeCommand  extends NewtonCommand{
    public HumanPlayerIntakeCommand(Intake intake){
        super(
            pivot.commands.setPivotPositionCommand(Positions.HP_LOAD).andThen(
                intake.intakeCommand(),intake.stopCommand()            )
        );
    }
}
