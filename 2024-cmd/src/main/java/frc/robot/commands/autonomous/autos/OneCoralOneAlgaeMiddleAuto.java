package frc.robot.commands.autonomous.autos;

import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class OneCoralOneAlgaeMiddleAuto extends AutoCommand{
    public OneCoralOneAlgaeMiddleAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("MiddleToDRight"), Suppliers.isRedAlliance, "")
            .alongWith(scoring.goToPosition(ElevatorPositions.getL4()))
            .andThen(scoring.outtakeCoralCommand().withTimeout(1)),
            new FollowPathCommand(getChoreoTrajectory("DRightToMiddle"), Suppliers.isRedAlliance, "")
            .alongWith(scoring.goToPosition(ElevatorPositions.getL2Algae()))
            .andThen(scoring.intakeUntilHasCoralCommand()),         
            new FollowPathCommand(getChoreoTrajectory("MiddleToDMiddle"), Suppliers.isRedAlliance, "")
            .andThen(scoring.outtakeCoralCommand().withTimeout(1)),
            new FollowPathCommand(getChoreoTrajectory("DMiddleBackUp"), Suppliers.isRedAlliance, "")
            .andThen(scoring.goToPosition(ElevatorPositions.getStow()))                       
        );

    }
}
