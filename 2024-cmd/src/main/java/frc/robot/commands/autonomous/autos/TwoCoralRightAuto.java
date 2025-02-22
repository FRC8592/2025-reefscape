package frc.robot.commands.autonomous.autos;

import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class TwoCoralRightAuto extends AutoCommand{
    public TwoCoralRightAuto(){
        super(
            //TODO: add intake and outtake commands.
            new FollowPathCommand(getChoreoTrajectory("RightToCLeft"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToPosition(ElevatorPositions.L4))
            .andThen(scoring.outtakeCommand()),
            new FollowPathCommand(getChoreoTrajectory("CLeftToHPRight"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToPosition(ElevatorPositions.STOW))
            .andThen(scoring.intakeUntilHasCoralCommand()),
            new FollowPathCommand(getChoreoTrajectory("HPRightToBLeft"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToPosition(ElevatorPositions.L4))
            .andThen(scoring.outtakeCommand()),
            new FollowPathCommand(getChoreoTrajectory("BLeftBackUp"), Suppliers.robotRunningOnRed)
            .andThen(scoring.goToPosition(ElevatorPositions.STOW))
        );

        setStartStateFromChoreoTrajectory("RightToCLeft");
    }
}
