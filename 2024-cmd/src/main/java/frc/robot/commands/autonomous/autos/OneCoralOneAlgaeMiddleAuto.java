package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class OneCoralOneAlgaeMiddleAuto extends AutoCommand{
    public OneCoralOneAlgaeMiddleAuto(DriverStation.Alliance color){
        super(
            new FollowPathCommand(getChoreoTrajectory("MiddleToDRight"+color.name()), Suppliers.isRedAlliance, "")
            .alongWith(scoring.goToPosition(ElevatorPositions.getL4()))
            .andThen(scoring.outtakeCoralCommand().withTimeout(0.2).asProxy()),
            scoring.setAlgaeMode(),
            new FollowPathCommand(getChoreoTrajectory("DRightToMiddle"+color.name()), Suppliers.isRedAlliance, "")
            .andThen(scoring.goToPosition(ElevatorPositions.getL2Algae())),
            new FollowPathCommand(getChoreoTrajectory("MiddleToDMiddle"), Suppliers.isRedAlliance, "")
            .andThen(scoring.intakeCommand().withTimeout(1).asProxy()),
            new FollowPathCommand(getChoreoTrajectory("DAlgaeTo"+color.name()+"Net"), Suppliers.isRedAlliance, "")
            .andThen(scoring.goToPosition(ElevatorPositions.getNet()))
            .andThen(scoring.outtakeCoralCommand().withTimeout(1).asProxy())
            .andThen(scoring.setCoralMode())
            .andThen(scoring.goToPosition(ElevatorPositions.getStow())), 
            new FollowPathCommand(getChoreoTrajectory(color.name()+"NetBackUp"), Suppliers.isRedAlliance, "")
            // .andThen(scoring.goToPosition(ElevatorPositions.getL3Algae())),
            // new FollowPathCommand(getChoreoTrajectory("DAlgaeTo"+color.name()+"Net"), Suppliers.isRedAlliance, "")
            // .andThen(scoring.goToPosition(ElevatorPositions.getNet()))
            // .andThen(scoring.outtakeCoralCommand().withTimeout(1))
            // .andThen(scoring.setCoralMode())
            // .andThen(scoring.goToPosition(ElevatorPositions.getStow()).withTimeout(1)),
            // new FollowPathCommand(getChoreoTrajectory(color.name()+"NetBackUp"), Suppliers.isRedAlliance, "")
            // .alongWith(scoring.goToPosition(ElevatorPositions.getStow()))           
        );

    }
}
