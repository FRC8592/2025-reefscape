package frc.robot.commands;

import frc.robot.commands.proxies.NewtonCommand;

public class HumanPlayerIntakeCommand  extends NewtonCommand{
    public HumanPlayerIntakeCommand(){
        super(
            pivot.commands.setPivotPositionCommand(Positions.HP_LOAD).andThen(
                intake.commands.intakeCommand(), stopSubsystems(intake.commands)
            )
        );
    }
}
