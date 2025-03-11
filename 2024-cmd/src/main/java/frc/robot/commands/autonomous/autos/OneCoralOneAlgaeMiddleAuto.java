package frc.robot.commands.autonomous.autos;

import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class OneCoralOneAlgaeMiddleAuto extends AutoCommand{
    public OneCoralOneAlgaeMiddleAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("MiddleToDRight"), Suppliers.robotRunningOnRed, "")
            .alongWith(scoring.goToPosition(ElevatorPositions.getL4()))
            .andThen(scoring.outtakeCoralCommand().withTimeout(1)),
            new FollowPathCommand(getChoreoTrajectory("DRightToMiddle"), Suppliers.robotRunningOnRed, "")
            .alongWith(scoring.goToPosition(ElevatorPositions.getL2Algae()))
            .andThen(scoring.intakeUntilHasCoralCommand()),         
            new FollowPathCommand(getChoreoTrajectory("MiddleToDMiddle"), Suppliers.robotRunningOnRed, "")
            .andThen(scoring.outtakeCoralCommand().withTimeout(1)),
            new FollowPathCommand(getChoreoTrajectory("DMiddleBackUp"), Suppliers.robotRunningOnRed, "")
            .andThen(scoring.goToPosition(ElevatorPositions.getStow()))                       
        );

    }
}
