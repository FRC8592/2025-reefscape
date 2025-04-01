package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

public class AllAlgaeAuto extends AutoCommand{
    public AllAlgaeAuto(){
        super(
            scoring.setCoralMode(),
            scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4()),
            new WaitUntilCommand(()->scoring.atPosition()),

            new FollowPathCommand(
                getChoreoTrajectory("MiddleToDLeft"),
                Suppliers.isRedAlliance,
                "",
                1.5,
                false,
                false
            )
            .andThen(new WaitCommand(0.75), scoring.outtakeCoralCommand().withTimeout(0.25)),

            new FollowPathCommand(
                getChoreoTrajectory("DLeftToDAlgae"),
                Suppliers.isRedAlliance,
                "",
                0.5,
                false,
                false
            ),

            new FollowPathCommand(
                getChoreoTrajectory("DAlgaeBackUp"),
                Suppliers.isRedAlliance,
                "",
                0.5,
                false,
                false
            )
            .alongWith(new WaitCommand(0.25).andThen(
                scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL2Algae())
            )),
            
            scoring.setAlgaeMode(),

            new FollowPathCommand(
                getChoreoTrajectory("MiddleToDAlgae"),
                Suppliers.isRedAlliance,
                "",
                0.5,
                false,
                false
            ).andThen(scoring.intakeCommand().withTimeout(2)),

            new FollowPathCommand(
                getChoreoTrajectory("DAlgaeToBlueNet"),
                Suppliers.isRedAlliance,
                "",
                0.5,
                false,
                false
            )
            .alongWith(new WaitCommand(0.25).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getNet())))
            .andThen(new WaitCommand(0.75), scoring.outtakeCoralCommand().withTimeout(0.25))

            // new FollowPathCommand(
            //     getChoreoTrajectory("BlueNetToEAlgae"),
            //     Suppliers.isRedAlliance,
            //     "",
            //     0.5,
            //     false,
            //     false
            // )
            // .alongWith(new WaitCommand(0.25).andThen(
            //     scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL2Algae()),
            //     scoring.intakeCommand().withTimeout(2)
            // )),

            // new FollowPathCommand(
            //     getChoreoTrajectory("EAlgaeToBlueNet"),
            //     Suppliers.isRedAlliance,
            //     "",
            //     0.5,
            //     false,
            //     false
            // )
            // .alongWith(new WaitCommand(0.25).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getNet())))
            // .andThen(new WaitCommand(0.75), scoring.outtakeCoralCommand().withTimeout(0.25)),

            // new FollowPathCommand(
            //     getChoreoTrajectory("BlueNetToCAlgae"),
            //     Suppliers.isRedAlliance,
            //     "",
            //     0.5,
            //     false,
            //     false
            // )
            // .alongWith(new WaitCommand(0.25).andThen(
            //     scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL2Algae()),
            //     scoring.intakeCommand().withTimeout(2)
            // )),

            // new FollowPathCommand(
            //     getChoreoTrajectory("CAlgaeToBlueNet"),
            //     Suppliers.isRedAlliance,
            //     "",
            //     0.5,
            //     false,
            //     false
            // )
            // .alongWith(new WaitCommand(0.25).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getNet())))
            // .andThen(new WaitCommand(0.75), scoring.outtakeCoralCommand().withTimeout(0.25))
        );

        setStartStateFromChoreoTrajectory("LeftToCMid");

    }
}
