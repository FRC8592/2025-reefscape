package frc.robot.commands.autonomous.autos;

import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class ThreeCoralLeftAuto extends AutoCommand{
    public ThreeCoralLeftAuto(){
        super(
            //TODO: add intake and outtake commands.
            new FollowPathCommand(getChoreoTrajectory("LeftToERight"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToPosition(ElevatorPositions.L4))
            .andThen(scoring.outtakeCommand()),
            new FollowPathCommand(getChoreoTrajectory("ERightToHPLeft"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToPosition(ElevatorPositions.STOW))
            .andThen(scoring.intakeUntilHasCoralCommand()),
            new FollowPathCommand(getChoreoTrajectory("HPLeftToFLeft"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToPosition(ElevatorPositions.L4))
            .andThen(scoring.outtakeCommand()),
            new FollowPathCommand(getChoreoTrajectory("FLeftToHPLeft"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToPosition(ElevatorPositions.STOW))
            .andThen(scoring.intakeUntilHasCoralCommand()),
            new FollowPathCommand(getChoreoTrajectory("HPLeftToFRight"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToPosition(ElevatorPositions.L4))
            .andThen(scoring.outtakeCommand()),
            new FollowPathCommand(getChoreoTrajectory("FRightBackUp"), Suppliers.robotRunningOnRed)
            .andThen(scoring.goToPosition(ElevatorPositions.STOW))
        );

        setStartStateFromChoreoTrajectory("LeftToERight");

    }
}
