package frc.robot.commands.autonomous.autos;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.ScoreCoral.LeftOrRight;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class ThreeCoralBlueAuto extends AutoCommand{
    public ThreeCoralBlueAuto(){
        super(
            (
                new FollowPathCommand(getChoreoTrajectory("LeftToERight"), Suppliers.isRedAlliance, "")
                .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(Suppliers.isRedAlliance.getAsBoolean()?LeftOrRight.Left:LeftOrRight.Right);}))
                .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
            )
            .alongWith(scoring.goToPosition(ElevatorPositions.getL4()))
            .andThen(new WaitCommand(0.75), scoring.outtakeCoralCommand().withTimeout(0.75)),
            new FollowPathCommand(getChoreoTrajectory("ERightToHPLeft"), Suppliers.isRedAlliance, "")
            .alongWith(new WaitCommand(1).andThen(scoring.goToPosition(ElevatorPositions.getStow())))
            .andThen(scoring.intakeUntilHasCoralCommand()),
            (
                new FollowPathCommand(getChoreoTrajectory("HPLeftToFLeft"), Suppliers.isRedAlliance, "")
                .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(Suppliers.isRedAlliance.getAsBoolean()?LeftOrRight.Right:LeftOrRight.Left);}))
                .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
            )
            .alongWith(new WaitCommand(1).andThen(scoring.goToPosition(ElevatorPositions.getL4())))
            .andThen(new WaitCommand(0.75), scoring.outtakeCoralCommand().withTimeout(0.75)),
            new FollowPathCommand(getChoreoTrajectory("FLeftToHPLeft"), Suppliers.isRedAlliance, "")
            .alongWith(new WaitCommand(1).andThen(scoring.goToPosition(ElevatorPositions.getStow())))
            .andThen(scoring.intakeUntilHasCoralCommand()),
            (
                new FollowPathCommand(getChoreoTrajectory("HPLeftToFRight"), Suppliers.isRedAlliance, "")
                .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(Suppliers.isRedAlliance.getAsBoolean()?LeftOrRight.Left:LeftOrRight.Right);}))
                .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
            )
            .alongWith(new WaitCommand(1).andThen(scoring.goToPosition(ElevatorPositions.getL4())))
            .andThen(new WaitCommand(0.75), scoring.outtakeCoralCommand().withTimeout(0.75)),
            new FollowPathCommand(getChoreoTrajectory("FRightBackUp"), Suppliers.isRedAlliance, "")
            .andThen(scoring.goToPosition(ElevatorPositions.getStow()))
        );


    }
}
