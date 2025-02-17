package frc.robot.commands.autonomous.autos;

import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class TwoCoralLeftAuto extends AutoCommand{
    public TwoCoralLeftAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("LeftToERight"), Suppliers.robotRunningOnRed),
            // .alongWith(scoring.goToSpecifiedPosition(ElevatorPositions.L4)),
            new FollowPathCommand(getChoreoTrajectory("ERightToHPLeft"), Suppliers.robotRunningOnRed),
            // .alongWith(scoring.stowCommand()),
            new FollowPathCommand(getChoreoTrajectory("HPLeftToFRight"), Suppliers.robotRunningOnRed),
            // .alongWith(scoring.goToSpecifiedPosition(ElevatorPositions.L4)),
            new FollowPathCommand(getChoreoTrajectory("FRightBackUp"), Suppliers.robotRunningOnRed)
            // .andThen(scoring.stowCommand())
        );

        setStartStateFromChoreoTrajectory("LeftToERight");
    }
}
