// package frc.robot.commands.autonomous.autos;

// import java.util.Set;

// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.Suppliers;
// import frc.robot.commands.autonomous.AutoCommand;
// import frc.robot.commands.largecommands.FollowPathCommand;
// import frc.robot.subsystems.ScoreCoral.LeftOrRight;
// import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;

// public class FourCoralBlueAuto extends AutoCommand{
//     public FourCoralBlueAuto(){
//         super(
//             (
//                 new FollowPathCommand(getChoreoTrajectory(redorblue?"LeftToERight":"Somethingelse", 0), Suppliers.robotRunningOnRed)
//                 .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(Suppliers.robotRunningOnRed.getAsBoolean()?LeftOrRight.Left:LeftOrRight.Right);}))
//                 .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
//             ).alongWith(scoring.goToPosition(ElevatorPositions.getL4()))
//             .andThen(new WaitCommand(0.25), scoring.outtakeCoralCommand().withTimeout(0.75)),

//             new FollowPathCommand(getChoreoTrajectory("ERightToHPLeft"), Suppliers.robotRunningOnRed)
//             .alongWith(new WaitCommand(1).andThen(scoring.goToPosition(ElevatorPositions.getStow()))),
//             // .andThen(scoring.intakeUntilHasCoralCommand()),

//             (
//                 new FollowPathCommand(getChoreoTrajectory("HPLeftToFLeft", 0), Suppliers.robotRunningOnRed)
//                 .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(Suppliers.robotRunningOnRed.getAsBoolean()?LeftOrRight.Right:LeftOrRight.Left);}))
//                 .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
//             )
//             .alongWith(new WaitCommand(1).andThen(scoring.goToPosition(ElevatorPositions.getL4())))
//             .andThen(new WaitCommand(0.25), scoring.outtakeCoralCommand().withTimeout(0.75)),

//             new FollowPathCommand(getChoreoTrajectory("FLeftToHPLeft", 0), Suppliers.robotRunningOnRed)
//             .alongWith(new WaitCommand(1).andThen(scoring.goToPosition(ElevatorPositions.getStow()))),
//             // .andThen(scoring.intakeUntilHasCoralCommand()),

//             (
//                 new FollowPathCommand(getChoreoTrajectory("HPLeftToFRight", 0), Suppliers.robotRunningOnRed)
//                 .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(Suppliers.robotRunningOnRed.getAsBoolean()?LeftOrRight.Left:LeftOrRight.Right);}))
//                 .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
//             )
//             .alongWith(new WaitCommand(1).andThen(scoring.goToPosition(ElevatorPositions.getL4())))
//             .andThen(new WaitCommand(0.25), scoring.outtakeCoralCommand().withTimeout(0.75)),

//             new FollowPathCommand(getChoreoTrajectory("FRightToHPLeft"), Suppliers.robotRunningOnRed)
//             .alongWith(new WaitCommand(1).andThen(scoring.goToPosition(ElevatorPositions.getStow()))),
//             // .andThen(scoring.intakeUntilHasCoralCommand()),

//             (
//                 new FollowPathCommand(getChoreoTrajectory("HPLeftToALeft", 0), Suppliers.robotRunningOnRed)
//                 .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(Suppliers.robotRunningOnRed.getAsBoolean()?LeftOrRight.Right:LeftOrRight.Left);}))
//                 .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
//             )
//             .alongWith(new WaitCommand(1).andThen(scoring.goToPosition(ElevatorPositions.getL4())))
//             .andThen(new WaitCommand(0.25), scoring.outtakeCoralCommand().withTimeout(0.75)),
            
//             new FollowPathCommand(getChoreoTrajectory("ALeftBackUp"), Suppliers.robotRunningOnRed)
//             .andThen(scoring.goToPosition(ElevatorPositions.getStow()))
//         );

//     }

// }
