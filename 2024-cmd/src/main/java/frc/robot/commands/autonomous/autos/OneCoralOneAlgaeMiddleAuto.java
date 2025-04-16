package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class OneCoralOneAlgaeMiddleAuto extends AutoCommand{
    public OneCoralOneAlgaeMiddleAuto(DriverStation.Alliance color){
        super(
            new FollowPathCommand(getChoreoTrajectory("MiddleToDRight"+color.name()), Suppliers.isRedAlliance, "")
            .alongWith(scoring.goToPosition(ElevatorPositions.getL4()))
            .andThen(Commands.waitSeconds(0.25), scoring.outtakeCoralCommand().withTimeout(0.2)),
        
            scoring.setAlgaeMode(),

            new FollowPathCommand(getChoreoTrajectory("DRightToMiddle"+color.name()), Suppliers.isRedAlliance, "")
            .andThen(scoring.goToPosition(ElevatorPositions.getL2Algae())),

            new FollowPathCommand(getChoreoTrajectory("MiddleToDMiddle"), Suppliers.isRedAlliance, "")
            .andThen(scoring.intakeCommand().withTimeout(0.5)),

            new FollowPathCommand(getChoreoTrajectory("DAlgaeTo"+color.name()+"Net"), Suppliers.isRedAlliance, "")
            .deadlineFor(scoring.intakeCommand(), Commands.waitSeconds(0.5).andThen(scoring.goToPosition(ElevatorPositions.getNet()))),

            new WaitUntilCommand(() -> scoring.atPosition())
            .deadlineFor(scoring.intakeCommand()),

            scoring.outtakeCoralCommand().withTimeout(1),

            scoring.setCoralMode(),

            scoring.goToPosition(ElevatorPositions.getStow()), 
            
            new FollowPathCommand(getChoreoTrajectory(color.name()+"NetBackUp"), Suppliers.isRedAlliance, "")       
        );

    }
}
