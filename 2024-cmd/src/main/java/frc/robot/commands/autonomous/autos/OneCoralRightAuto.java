package frc.robot.commands.autonomous.autos;

import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class OneCoralRightAuto extends AutoCommand{
    public OneCoralRightAuto(){
        super(
            //TODO: add intake and outtake commands.
            new FollowPathCommand(getChoreoTrajectory("RightToCLeft"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToSpecifiedPosition(ElevatorPositions.L4)),
            new FollowPathCommand(getChoreoTrajectory("CLeftBackUp"), Suppliers.robotRunningOnRed)
            .andThen(scoring.stowCommand())
        );

        setStartStateFromChoreoTrajectory("RightToCLeft");

    }
}
