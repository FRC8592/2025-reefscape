package frc.robot.commands;

import frc.robot.commands.proxies.NewtonCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot.Positions;

public class GroundIntakeCommand extends NewtonCommand{
    public GroundIntakeCommand(Intake intake){
        super(
            pivot.commands.setPivotPositionCommand(Positions.GROUND).andThen(
                intake.intakeCommand(),intake.stopCommand()
            )
        );
    }
}
