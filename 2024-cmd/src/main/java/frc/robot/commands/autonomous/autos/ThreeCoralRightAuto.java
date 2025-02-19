package frc.robot.commands.autonomous.autos;

import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class ThreeCoralRightAuto extends AutoCommand{
    public ThreeCoralRightAuto(){
        super(
            //TODO: add intake and outtake commands.
            new FollowPathCommand(getChoreoTrajectory("RightToCLeft"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToSpecifiedPosition(ElevatorPositions.L4))
            .andThen(scoring.outtakeCommand()),
            new FollowPathCommand(getChoreoTrajectory("CLeftToHPRight"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.stowCommand())
            .andThen(scoring.intakeCommand()),
            new FollowPathCommand(getChoreoTrajectory("HPRightToBRight"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToSpecifiedPosition(ElevatorPositions.L4))
            .andThen(scoring.outtakeCommand()),
            new FollowPathCommand(getChoreoTrajectory("BRightToHPRight"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.stowCommand())
            .andThen(scoring.intakeCommand()),
            new FollowPathCommand(getChoreoTrajectory("HPRightToBLeft"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToSpecifiedPosition(ElevatorPositions.L4)).
            andThen(scoring.outtakeCommand()),
            new FollowPathCommand(getChoreoTrajectory("BLeftBackUp"), Suppliers.robotRunningOnRed)
            .andThen(scoring.stowCommand())
        );

        setStartStateFromChoreoTrajectory("RightToCLeft");
    }
}
