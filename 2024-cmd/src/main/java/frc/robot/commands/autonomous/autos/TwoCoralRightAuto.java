package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class TwoCoralRightAuto extends AutoCommand{
    public TwoCoralRightAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("RightToCLeft"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToPosition(ElevatorPositions.getL4()))
            .andThen(scoring.outtakeCommand().withTimeout(1)),
            new FollowPathCommand(getChoreoTrajectory("CLeftToHPRight"), Suppliers.robotRunningOnRed)
            .alongWith((new WaitCommand(1).andThen(scoring.goToPosition(ElevatorPositions.getStow()))))
            .andThen(scoring.intakeUntilHasCoralCommand()),
            new FollowPathCommand(getChoreoTrajectory("HPRightToBLeft"), Suppliers.robotRunningOnRed)
            .alongWith((new WaitCommand(1).andThen(scoring.goToPosition(ElevatorPositions.getL4())))
            .andThen(scoring.outtakeCommand().withTimeout(1)),
            new FollowPathCommand(getChoreoTrajectory("BLeftBackUp"), Suppliers.robotRunningOnRed)
            .andThen(scoring.goToPosition(ElevatorPositions.getStow()))
        ));

        setStartStateFromChoreoTrajectory("RightToCLeft");
    }
}
