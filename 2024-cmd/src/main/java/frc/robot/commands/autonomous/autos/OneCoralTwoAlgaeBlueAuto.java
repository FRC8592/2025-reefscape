package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class OneCoralTwoAlgaeBlueAuto extends AutoCommand{
    public OneCoralTwoAlgaeBlueAuto(DriverStation.Alliance color){
        super(
            new FollowPathCommand(getChoreoTrajectory("MiddleToDRight"+color.name()), Suppliers.isRedAlliance, "")
            .alongWith(scoring.goToPosition(ElevatorPositions.getL4()))
            .andThen(scoring.outtakeCoralCommand().withTimeout(0.2)),
            scoring.setAlgaeMode(),
            new FollowPathCommand(getChoreoTrajectory("DRightToMiddle"+color.name()), Suppliers.isRedAlliance, "")
            .andThen(scoring.goToPosition(ElevatorPositions.getL2Algae()).withTimeout(0.5)),
            new FollowPathCommand(getChoreoTrajectory("MiddleToDMiddle"), Suppliers.isRedAlliance, ""),
            new FollowPathCommand(getChoreoTrajectory("DAlgaeTo"+color.name()+"Net"), Suppliers.isRedAlliance, "")
            .andThen(scoring.goToPosition(ElevatorPositions.getNet()))
            .andThen(scoring.outtakeCoralCommand().withTimeout(0.5))
            .andThen(scoring.goToPosition(ElevatorPositions.getL3Algae())),
            new FollowPathCommand(getChoreoTrajectory(color.name()+"NetToEAlgae"), Suppliers.isRedAlliance, ""),
            new FollowPathCommand(getChoreoTrajectory("EAlgaeTo"+color.name()+"Net"), Suppliers.isRedAlliance, "")
            .andThen(scoring.goToPosition(ElevatorPositions.getNet()))
            .andThen(scoring.outtakeCoralCommand().withTimeout(0.5))
            .andThen(scoring.setCoralMode())
            .andThen(scoring.goToPosition(ElevatorPositions.getStow())),
            new FollowPathCommand(getChoreoTrajectory(color.name()+"NetBackUp"), Suppliers.isRedAlliance, "")
        );

    }
}
