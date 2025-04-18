package frc.robot.commands.autonomous.autos;

import java.util.Set;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class OneCoralTwoAlgaeRedAuto extends AutoCommand{
    public OneCoralTwoAlgaeRedAuto(DriverStation.Alliance color){
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

            // new WaitUntilCommand(() -> scoring.atPosition())
            // .deadlineFor(scoring.intakeCommand()),

            
            scoring.outtakeCoralCommand().withTimeout(1),

            scoring.setAlgaeMode(),

            Commands.defer(
                () -> (
                    Suppliers.isRedAlliance.getAsBoolean()
                    ? (new FollowPathCommand(getChoreoTrajectory("RedNetToCAlgae"), Suppliers.isRedAlliance, "").alongWith(new WaitCommand(0.5).andThen(scoring.goToPosition(ElevatorPositions.getL3Algae()))).andThen(scoring.intakeCommand().withTimeout(0.5)))
                    : (new FollowPathCommand(getChoreoTrajectory("BlueNetToEAlgae"), Suppliers.isRedAlliance, "").alongWith(new WaitCommand(0.5).andThen(scoring.goToPosition(ElevatorPositions.getL3Algae()))).andThen(scoring.intakeCommand().withTimeout(0.5)))
                ),
                Set.of(swerve)

            ),

            Commands.defer(
                () -> (
                    Suppliers.isRedAlliance.getAsBoolean()
                    ? (new FollowPathCommand(getChoreoTrajectory("CAlgaeToRedNet"), Suppliers.isRedAlliance, "").deadlineFor(scoring.intakeCommand(), Commands.waitSeconds(0.5).andThen(scoring.goToPosition(ElevatorPositions.getNet()))))
                    : (new FollowPathCommand(getChoreoTrajectory("EAlgaeToBlueNet"), Suppliers.isRedAlliance, "").deadlineFor(scoring.intakeCommand(), Commands.waitSeconds(0.5).andThen(scoring.goToPosition(ElevatorPositions.getNet()))))
                ),
            Set.of(swerve)
            ),

            new WaitUntilCommand(() -> scoring.atPosition())
            .deadlineFor(scoring.intakeCommand()),

            scoring.outtakeCoralCommand().withTimeout(1),

            scoring.setCoralMode(), 
            
            new FollowPathCommand(getChoreoTrajectory(color.name()+"NetBackUp"), Suppliers.isRedAlliance, "")
            .alongWith(new WaitCommand(0.5).andThen(scoring.goToPosition(ElevatorPositions.getStow())))
            
            
            
            
             
        );

    }
}
