package frc.robot.commands.autonomous.autos;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.ScoreCoral.LeftOrRight;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class OneCoralRedAuto extends AutoCommand{
    public OneCoralRedAuto(){
        super(
            (
                new FollowPathCommand(getChoreoTrajectory("RightToCLeft"), Suppliers.robotRunningOnRed)
                .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(Suppliers.robotRunningOnRed.getAsBoolean()?LeftOrRight.Right:LeftOrRight.Left);}))
                .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
            )
            .alongWith(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4()))
            .andThen(intake.setIntakeCommand(-0.43).withTimeout(1)),
            
            new FollowPathCommand(getChoreoTrajectory("CLeftBackUp"), Suppliers.robotRunningOnRed)
            .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow()))
        ));


    }
}
