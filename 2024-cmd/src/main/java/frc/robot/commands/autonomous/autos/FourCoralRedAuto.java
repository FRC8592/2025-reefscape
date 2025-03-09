package frc.robot.commands.autonomous.autos;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.ScoreCoral.LeftOrRight;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class FourCoralRedAuto extends AutoCommand{
    public FourCoralRedAuto(){
        super(
            // NOTE: DTT = drive-to-tag
            ( // Move from our start position to the reef, cutting the path off in the middle to activate DTT
                new FollowPathCommand(getChoreoTrajectory("RightToCLeft", 0), Suppliers.robotRunningOnRed).withTimeout(3.25)
                .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(Suppliers.robotRunningOnRed.getAsBoolean()?LeftOrRight.Right:LeftOrRight.Left);}))
                .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
            )
            // While running path and DTT, raise the scoring mech to L4 position
            .alongWith(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4()))
            // Once both the path and scoring mechanism are finished, score the first coral
            .andThen(new WaitCommand(0.25), scoring.outtakeCoralCommand().withTimeout(0.75)),

            // Move from the reef to the human player station
            new FollowPathCommand(getChoreoTrajectory("CLeftToHPRight"), Suppliers.robotRunningOnRed)
            // While moving, stow (after waiting a moment to clear the reef)
            .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow())))
            // Once we're stowed and at the human player station, intake
            .andThen(scoring.intakeUntilHasCoralCommand()),

            ( // Move from our start position to the reef, cutting the path off in the middle to activate DTT
                new FollowPathCommand(getChoreoTrajectory("HPRightToBRight", 0), Suppliers.robotRunningOnRed).withTimeout(3.25)
                .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(Suppliers.robotRunningOnRed.getAsBoolean()?LeftOrRight.Right:LeftOrRight.Left);}))
                .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
            )
            // While running path and DTT, raise the scoring mech to L4 position
            .alongWith(new WaitCommand(0.5).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4())))
            // Once both the path and scoring mechanism are finished, score the first coral
            .andThen(new WaitCommand(0.25), scoring.outtakeCoralCommand().withTimeout(0.75)),

            // Move from the reef to the human player station
            new FollowPathCommand(getChoreoTrajectory("BRightToHPRight"), Suppliers.robotRunningOnRed)
            // While moving, stow (after waiting a moment to clear the reef)
            .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow())))
            // Once we're stowed and at the human player station, intake
            .andThen(scoring.intakeUntilHasCoralCommand()),

            ( // Move from our start position to the reef, cutting the path off in the middle to activate DTT
                new FollowPathCommand(getChoreoTrajectory("HPRightToBLeft", 0), Suppliers.robotRunningOnRed)
                .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(Suppliers.robotRunningOnRed.getAsBoolean()?LeftOrRight.Right:LeftOrRight.Left);}))
                .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
            )
            // While running path and DTT, raise the scoring mech to L4 position
            .alongWith(new WaitCommand(0.5).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4())))
            // Once both the path and scoring mechanism are finished, score the first coral
            .andThen(new WaitCommand(0.25), scoring.outtakeCoralCommand().withTimeout(0.75)),

            // Move from the reef to the human player station
            new FollowPathCommand(getChoreoTrajectory("BLeftToHPRight"), Suppliers.robotRunningOnRed)
            // While moving, stow (after waiting a moment to clear the reef)
            .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow())))
            // Once we're stowed and at the human player station, intake
            .andThen(scoring.intakeUntilHasCoralCommand()),

            ( // Move from our start position to the reef, cutting the path off in the middle to activate DTT
                new FollowPathCommand(getChoreoTrajectory("HPRightToARight", 0), Suppliers.robotRunningOnRed)
                .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(Suppliers.robotRunningOnRed.getAsBoolean()?LeftOrRight.Right:LeftOrRight.Left);}))
                .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
            )
            // While running path and DTT, raise the scoring mech to L4 position
            .alongWith(new WaitCommand(0.5).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4())))
            // Once both the path and scoring mechanism are finished, score the first coral
            .andThen(new WaitCommand(0.25), scoring.outtakeCoralCommand().withTimeout(0.75)),

            new FollowPathCommand(getChoreoTrajectory("ARightBackUp"), Suppliers.robotRunningOnRed)
            .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow())))
            
            );

    }

}

//.alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow()))
