package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class FourCoralLeftAuto extends AutoCommand{
    public FourCoralLeftAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("LeftToERight"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToPosition(ElevatorPositions.getL4()))
            .andThen(scoring.outtakeCommand().withTimeout(1)),
            new FollowPathCommand(getChoreoTrajectory("ERightToHPLeft"), Suppliers.robotRunningOnRed)
            .alongWith(new WaitCommand(1).andThen(scoring.goToPosition(ElevatorPositions.getStow())))
            .andThen(scoring.intakeUntilHasCoralCommand()),
            new FollowPathCommand(getChoreoTrajectory("HPLeftToFLeft"), Suppliers.robotRunningOnRed)
            .alongWith(new WaitCommand(1).andThen(scoring.goToPosition(ElevatorPositions.getL4())))
            .andThen(scoring.outtakeCommand().withTimeout(1)),
            new FollowPathCommand(getChoreoTrajectory("FLeftToHPLeft"), Suppliers.robotRunningOnRed)
            .alongWith(new WaitCommand(1).andThen(scoring.goToPosition(ElevatorPositions.getStow())))
            .andThen(scoring.intakeUntilHasCoralCommand()),
            new FollowPathCommand(getChoreoTrajectory("HPLeftToFRight"), Suppliers.robotRunningOnRed)
            .alongWith(new WaitCommand(1).andThen(scoring.goToPosition(ElevatorPositions.getL4())))
            .andThen(scoring.outtakeCommand().withTimeout(1)),
            new FollowPathCommand(getChoreoTrajectory("FRightToHPLeft"), Suppliers.robotRunningOnRed)
            .alongWith(new WaitCommand(1).andThen(scoring.goToPosition(ElevatorPositions.getStow())))
            .andThen(scoring.intakeUntilHasCoralCommand()),
            new FollowPathCommand(getChoreoTrajectory("HPLeftToALeft"), Suppliers.robotRunningOnRed)
            .alongWith(new WaitCommand(1).andThen(scoring.goToPosition(ElevatorPositions.getL4())))
            .andThen(scoring.outtakeCommand().withTimeout(1)),
            new FollowPathCommand(getChoreoTrajectory("ALeftBackUp"), Suppliers.robotRunningOnRed)
            .andThen(scoring.goToPosition(ElevatorPositions.getStow()))
        );

        setStartStateFromChoreoTrajectory("LeftToERight");
    }

}
