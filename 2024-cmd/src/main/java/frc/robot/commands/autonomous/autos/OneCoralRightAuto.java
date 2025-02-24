package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class OneCoralRightAuto extends AutoCommand{
    public OneCoralRightAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("RightToCLeft"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.setUserPosition(ElevatorPositions.getL4()).andThen(scoring.applyUserPosition()))
            .andThen(intake.setIntakeCommand(-0.43).withTimeout(1)),
            new FollowPathCommand(getChoreoTrajectory("CLeftBackUp"), Suppliers.robotRunningOnRed)
            .alongWith(new WaitCommand(1).andThen(scoring.setUserPosition(ElevatorPositions.getStow()).andThen(scoring.applyUserPosition()))
        ));

        setStartStateFromChoreoTrajectory("RightToCLeft");

    }
}
