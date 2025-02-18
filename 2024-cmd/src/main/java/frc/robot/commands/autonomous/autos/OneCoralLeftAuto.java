package frc.robot.commands.autonomous.autos;

import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class OneCoralLeftAuto extends AutoCommand{
    public OneCoralLeftAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("LeftToERight"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToSpecifiedPosition(ElevatorPositions.L4))
            .andThen(scoring.outtakeCommand()),
            new FollowPathCommand(getChoreoTrajectory("ERightBackUp"), Suppliers.robotRunningOnRed)
            .andThen(scoring.stowCommand())           
        );

        setStartStateFromChoreoTrajectory("LeftToERight");

    }
}
