package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class OneCoralTwoAlgaeAuto extends AutoCommand{
    public OneCoralTwoAlgaeAuto(DriverStation.Alliance color){
        super(
            new FollowPathCommand(getChoreoTrajectory("MiddleToDRight"+color.name()), Suppliers.isRedAlliance, "")
            //new FollowPathCommand(getChoreoTrajectory("MiddleToDRightRed"), Suppliers.isRedAlliance, "")
            .alongWith(scoring.goToPosition(ElevatorPositions.getL4()))
            .andThen(scoring.outtakeCoralCommand().withTimeout(0.5)),
            scoring.setAlgaeMode(),
            new FollowPathCommand(getChoreoTrajectory("DRightToMiddle"+color.name()), Suppliers.isRedAlliance, "")
            //new FollowPathCommand(getChoreoTrajectory("DRightToMiddleRed"), Suppliers.isRedAlliance, "")
            .alongWith(scoring.goToPosition(ElevatorPositions.getL2Algae()).withTimeout(0.5)),
            new FollowPathCommand(getChoreoTrajectory("MiddleToDMiddle"), Suppliers.isRedAlliance, "")
            .andThen(scoring.intakeCommand().withTimeout(1)),
            ///new FollowPathCommand(getChoreoTrajectory("DAlgaeTo"+color.name()+"Net"), Suppliers.isRedAlliance, "")
            new FollowPathCommand(getChoreoTrajectory("DAlgaeToRedNet"), Suppliers.isRedAlliance, "")
            .alongWith(scoring.goToPosition(ElevatorPositions.getNet()))
            .andThen(scoring.outtakeCoralCommand().withTimeout(0.5)),
            //new FollowPathCommand(getChoreoTrajectory(color.name()+"NetToCAlgae"), Suppliers.isRedAlliance, "")
            new FollowPathCommand(getChoreoTrajectory("RedNetToCAlgae"), Suppliers.isRedAlliance, "")
            .alongWith(scoring.goToPosition(ElevatorPositions.getL3Algae()))
            .andThen(scoring.intakeCommand().withTimeout(0.5)),
            //new FollowPathCommand(getChoreoTrajectory("CAlgaeTo"+color.name()+"Net"), Suppliers.isRedAlliance, "")
            new FollowPathCommand(getChoreoTrajectory("CAlgaeToRedNet"), Suppliers.isRedAlliance, "")
            .alongWith(scoring.goToPosition(ElevatorPositions.getNet()))
            .andThen(scoring.outtakeCoralCommand().withTimeout(0.5)),
            scoring.setCoralMode(),
            //new FollowPathCommand(getChoreoTrajectory(color.name()+"NetBackUp"), Suppliers.isRedAlliance, "")
            new FollowPathCommand(getChoreoTrajectory("RedNetBackUp"), Suppliers.isRedAlliance, "")
            .alongWith(scoring.goToPosition(ElevatorPositions.getStow()))
        );

    }
}
