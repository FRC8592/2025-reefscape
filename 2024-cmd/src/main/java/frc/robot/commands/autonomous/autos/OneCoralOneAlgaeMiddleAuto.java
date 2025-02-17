package frc.robot.commands.autonomous.autos;

import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class OneCoralOneAlgaeMiddleAuto extends AutoCommand{
    public OneCoralOneAlgaeMiddleAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("MiddleToDRight"), Suppliers.robotRunningOnRed),
            //.alongWith(scoring.goToSpecifiedPosition(ElevatorPositions.L4)),
            new FollowPathCommand(getChoreoTrajectory("DRightToMiddle"), Suppliers.robotRunningOnRed),
            // .alongWith(scoring.goToSpecifiedPosition(ElevatorPositions.L2_ALGAE)),         
            new FollowPathCommand(getChoreoTrajectory("MiddleToDMiddle"), Suppliers.robotRunningOnRed),
            new FollowPathCommand(getChoreoTrajectory("DMiddleBackUp"), Suppliers.robotRunningOnRed)
            // .andThen(scoring.stowCommand())                       
        );

        setStartStateFromChoreoTrajectory("MiddleToDRight");
    }
}
