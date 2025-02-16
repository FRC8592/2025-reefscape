package frc.robot.commands.autonomous.autos;

import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class TwoCoralRightAuto extends AutoCommand{
    public TwoCoralRightAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("RightToCLeft"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToSpecifiedPosition(ElevatorPositions.L4)),
            new FollowPathCommand(getChoreoTrajectory("CLeftToHPRight"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.stowCommand()),
            new FollowPathCommand(getChoreoTrajectory("HPRightToBLeft"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToSpecifiedPosition(ElevatorPositions.L4)),
            new FollowPathCommand(getChoreoTrajectory("BLeftBackUp"), Suppliers.robotRunningOnRed)
            .andThen(scoring.stowCommand())
        );

        setStartStateFromChoreoTrajectory("RightToCLeft");
    }
}
