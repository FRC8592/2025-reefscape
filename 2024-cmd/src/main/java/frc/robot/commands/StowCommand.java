package frc.robot.commands;

import frc.robot.commands.proxies.NewtonCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot.Positions;

public class StowCommand extends NewtonCommand{
    public StowCommand(Intake intake){
        super(
            pivot.commands.setPivotPositionCommand(Positions.REST).alongWith(
                intake.intakeCommand(),intake.stopCommand()            )
        );
    }
}
