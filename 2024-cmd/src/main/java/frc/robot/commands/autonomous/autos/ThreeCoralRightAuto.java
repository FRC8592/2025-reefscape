package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class ThreeCoralRightAuto extends AutoCommand{
    public ThreeCoralRightAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("RightToCLeft"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4()))
            .andThen(intake.setIntakeCommand(-0.43).withTimeout(1)),
            
            new FollowPathCommand(getChoreoTrajectory("CLeftToHPRight"), Suppliers.robotRunningOnRed)
            .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow())))
            .andThen(intake.setIntakeCommand(0.5).withTimeout(1)),
            
            new FollowPathCommand(getChoreoTrajectory("HPRightToBRight"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4()))
            .andThen(intake.setIntakeCommand(-0.43).withTimeout(1)),
            
            new FollowPathCommand(getChoreoTrajectory("BRightToHPRight"), Suppliers.robotRunningOnRed)
            .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow())))
            .andThen(intake.setIntakeCommand(0.5).withTimeout(1)),
            
            new FollowPathCommand(getChoreoTrajectory("HPRightToBLeft"), Suppliers.robotRunningOnRed)
            .alongWith(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4()))
            .andThen(intake.setIntakeCommand(-0.43).withTimeout(1)),
            
            new FollowPathCommand(getChoreoTrajectory("BLeftBackUp"), Suppliers.robotRunningOnRed)
            .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow())))
        );

        setStartStateFromChoreoTrajectory("RightToCLeft");
    }
}
