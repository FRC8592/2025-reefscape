package frc.robot.commands.autonomous.autos;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.ScoreCoral.LeftOrRight;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class OneCoralBlueAuto extends AutoCommand{
    public OneCoralBlueAuto(){
        super(
            (
                new FollowPathCommand(getChoreoTrajectory("LeftToERight"), Suppliers.robotRunningOnRed, "")
                .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(Suppliers.robotRunningOnRed.getAsBoolean()?LeftOrRight.Left:LeftOrRight.Right);}))
                .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
            )
            .alongWith(scoring.goToPosition(ElevatorPositions.getL4()))
            .andThen(scoring.outtakeCoralCommand().withTimeout(1)),
            new FollowPathCommand(getChoreoTrajectory("ERightBackUp"), Suppliers.robotRunningOnRed, "")
            .andThen(scoring.goToPosition(ElevatorPositions.getStow()))           
        );


    }
}
