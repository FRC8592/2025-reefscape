package frc.robot.commands;

import frc.robot.commands.proxies.NewtonCommand;

public class GroundIntakeCommand extends NewtonCommand{
    public GroundIntakeCommand(){
        super(
            pivot.commands.setPivotPositionCommand(Positions.GROUND).andThen(
                intake.commands.intakeCommand(), stopSubsystems(intake.commands)
            )
        );
    }
}
