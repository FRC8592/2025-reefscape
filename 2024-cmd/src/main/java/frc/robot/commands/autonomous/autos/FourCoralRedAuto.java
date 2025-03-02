package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class FourCoralRedAuto extends AutoCommand{
    public FourCoralRedAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("RightToCLeft"), Suppliers.robotRunningOnRed).withTimeout(3.25)
            .alongWith(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4()))
            .andThen(new WaitCommand(0.75), scoring.outtakeCoralCommand().withTimeout(0.75)),

            new FollowPathCommand(getChoreoTrajectory("CLeftToHPRight"), Suppliers.robotRunningOnRed)
            .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow())))
            .andThen(scoring.intakeUntilHasCoralCommand()),

            new FollowPathCommand(getChoreoTrajectory("HPRightToBRight"), Suppliers.robotRunningOnRed).withTimeout(3.25)
            .alongWith(new WaitCommand(0.5).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4())))
            .andThen(new WaitCommand(0.75), scoring.outtakeCoralCommand().withTimeout(0.75)),

            new FollowPathCommand(getChoreoTrajectory("BRightToHPRight"), Suppliers.robotRunningOnRed)
            .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow())))
            .andThen(scoring.intakeUntilHasCoralCommand()),

            new FollowPathCommand(getChoreoTrajectory("HPRightToBLeft"), Suppliers.robotRunningOnRed)
            .alongWith(new WaitCommand(0.5).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4())))
            .andThen(new WaitCommand(0.75), scoring.outtakeCoralCommand().withTimeout(0.75)),

            new FollowPathCommand(getChoreoTrajectory("BLeftToHPRight"), Suppliers.robotRunningOnRed)
            .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow())))
            .andThen(scoring.intakeUntilHasCoralCommand()),

            new FollowPathCommand(getChoreoTrajectory("HPRightToARight"), Suppliers.robotRunningOnRed)
            .alongWith(new WaitCommand(0.5).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4())))
            .andThen(new WaitCommand(0.75), scoring.outtakeCoralCommand().withTimeout(0.75)),

            new FollowPathCommand(getChoreoTrajectory("ARightBackUp"), Suppliers.robotRunningOnRed)
            .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow())))
            
            );

        setStartStateFromChoreoTrajectory("RightToCLeft");
    }

}

//.alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow()))
