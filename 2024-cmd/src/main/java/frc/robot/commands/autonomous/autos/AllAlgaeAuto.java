package frc.robot.commands.autonomous.autos;

import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class AllAlgaeAuto extends AutoCommand{
    public AllAlgaeAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("LeftToCMid"), Suppliers.robotRunningOnRed)
                .alongWith(scoring.setUserPosition(ElevatorPositions.getL3Algae()))
                .andThen(scoring.outtakeCommand()),
            new FollowPathCommand(getChoreoTrajectory("CMidToBMid"), Suppliers.robotRunningOnRed)
                .alongWith(scoring.setUserPosition(ElevatorPositions.getL2Algae()))
                .andThen(scoring.outtakeCommand()),
            new FollowPathCommand(getChoreoTrajectory("BMidToAMid"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.setUserPosition(ElevatorPositions.getL3Algae()))
            .andThen(scoring.outtakeCommand())
        );

        setStartStateFromChoreoTrajectory("LeftToCMid");

    }
}
