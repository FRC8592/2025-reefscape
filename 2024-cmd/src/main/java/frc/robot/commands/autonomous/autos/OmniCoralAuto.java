package frc.robot.commands.autonomous.autos;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.OdometryUpdates;
import frc.robot.subsystems.ScoreCoral.LeftOrRight;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;
import static frc.robot.commands.autonomous.autos.OmniCoralAuto.Positions.*;

public class OmniCoralAuto extends AutoCommand{
    protected enum Positions{
        FIRST_CORAL_SCORE("RightToCLeft", LeftOrRight.Left, "LeftToERight", LeftOrRight.Right),
        SECOND_CORAL_INTAKE("CLeftToHPRight", LeftOrRight.Right, "ERightToHPLeft", LeftOrRight.Left),
        SECOND_CORAL_SCORE("HPRightToBRight", LeftOrRight.Right, "HPLeftToFLeft", LeftOrRight.Left),
        THIRD_CORAL_INTAKE("BRightToHPRight", LeftOrRight.Right, "FLeftToHPLeft", LeftOrRight.Left),
        THIRD_CORAL_SCORE("HPRightToBLeft", LeftOrRight.Left, "HPLeftToFRight", LeftOrRight.Right),
        FOURTH_CORAL_INTAKE("BLeftToHPRight", LeftOrRight.Right, "FRightToHPLeft", LeftOrRight.Left),
        FOURTH_CORAL_SCORE("HPRightToARight", LeftOrRight.Right, "HPLeftToALeft", LeftOrRight.Left),
        FOURTH_CORAL_BACK_UP("ARightBackUp", LeftOrRight.Left, "ALeftBackUp", LeftOrRight.Left)
        ;
        String red, blue;
        LeftOrRight redLeftOrRight;
        LeftOrRight blueLeftOrRight;
        private Positions(String redName, LeftOrRight redLeftOrRight, String blueName, LeftOrRight blueLeftOrRight){
            this.red = redName;
            this.blue = blueName;
            this.redLeftOrRight = redLeftOrRight;
            this.blueLeftOrRight = blueLeftOrRight;
        }
        public String getPathName(RedOrBlue redOrBlue){
            if(redOrBlue == RedOrBlue.RED){
                return red;
            }
            else{
                return blue;
            }
        }
        public LeftOrRight getLeftOrRight(RedOrBlue redOrBlueBarge){
            return (
                Suppliers.isRedAlliance.getAsBoolean()
                ? redOrBlueBarge == RedOrBlue.RED ? blueLeftOrRight : redLeftOrRight
                : redOrBlueBarge == RedOrBlue.RED ? redLeftOrRight : blueLeftOrRight
            );
        }
    }
    public enum RedOrBlue{
        RED,
        BLUE,;
    }
    public OmniCoralAuto(int coralCount, RedOrBlue redOrBlue){
        super(
            // NOTE: DTT = drive-to-tag
            coralCount > 0 ? ( // Score the first coral if coralcount > 0
                ( // Move from our start position to the reef, cutting the path off in the middle to activate DTT
                    new FollowPathCommand(
                        getChoreoTrajectory(FIRST_CORAL_SCORE.getPathName(redOrBlue)),//, 0),
                        Suppliers.isRedAlliance,
                        "FirstCoralScoreChoreoPath",
                        0.5,
                        false,
                        true
                    )
                    // .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(FIRST_CORAL_SCORE.getLeftOrRight(redOrBlue));}))
                    // .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
                )
                // While running path and DTT, raise the scoring mech to L4 position
                .alongWith(Commands.none().andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4())))
                // Once both the path and scoring mechanism are finished, score the first coral
                .andThen(new WaitCommand(0.25), scoring.outtakeCoralCommand().withTimeout(0.125))
            ):Commands.none(), // Commands.none() if coralcount < 1

            coralCount > 0 ? ( // Also intake the next coral if coralcount > 0
                // Move from the reef to the human player station
                new FollowPathCommand(getChoreoTrajectory(SECOND_CORAL_INTAKE.getPathName(redOrBlue)), Suppliers.isRedAlliance, "SecondCoralIntakeChoreoPath", 0.5, false, false)
                // While moving, stow (after waiting a moment to clear the reef)
                .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow())))
                // Once we're stowed and at the human player station, intake
                .andThen(scoring.intakeUntilHasCoralCommand())
            ):Commands.none(), // Do not intake if coralcount < 1

            coralCount > 1 ? ( // Score the second coral if coralcount > 1
                ( // Move from our start position to the reef, cutting the path off in the middle to activate DTT
                    new FollowPathCommand(
                        getChoreoTrajectory(SECOND_CORAL_SCORE.getPathName(redOrBlue)),//, 0),
                        Suppliers.isRedAlliance,
                        "SecondCoralScoreChoreoPath",
                        0.5,
                        false,
                        true
                    )
                    // Commands.none().andThen(Commands.runOnce(() -> {scoreCoral.setPosition(SECOND_CORAL_SCORE.getLeftOrRight(redOrBlue));}))
                    // .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
                )
                // While running path and DTT, raise the scoring mech to L4 position
                .alongWith(Commands.none().andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4())))
                // Once both the path and scoring mechanism are finished, score the first coral
                .andThen(scoring.outtakeCoralCommand().withTimeout(0.125))
            ):Commands.none(), // Don't do anything if coralcount < 2

            coralCount > 1 ? ( // Intake if we scored our second coral
                // Move from the reef to the human player stew WaitCommand(1).andThen(ation
                new FollowPathCommand(getChoreoTrajectory(THIRD_CORAL_INTAKE.getPathName(redOrBlue)), Suppliers.isRedAlliance,"ThirdCoralIntakeChoreoPath", 0.5, false, false)
                // While moving, stow (after waiting a moment to clear the reef)
                .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow())))
                // Once we're stowed and at the human player station, intake
                .andThen(scoring.intakeUntilHasCoralCommand())
            ):Commands.none(), // Don't do anything if we're not doing a second coral

            coralCount > 2 ? ( // Score the third coral if coralcount > 2
                ( // Move from our start position to the reef, cutting the path off in the middle to activate DTT
                    new FollowPathCommand(
                        getChoreoTrajectory(THIRD_CORAL_SCORE.getPathName(redOrBlue)),//, 0),
                        Suppliers.isRedAlliance,
                        "ThirdCoralScoreChoreoPath",
                        0.5,
                        false,
                        true
                    )
                    // .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(THIRD_CORAL_SCORE.getLeftOrRight(redOrBlue));}))
                    // .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
                )
                // While running path and DTT, raise the scoring mech to L4 position
                .alongWith(Commands.none().andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4())))
                // Once both the path and scoring mechanism are finished, score the first coral
                .andThen(scoring.outtakeCoralCommand().withTimeout(0.125))
            ):Commands.none(), // Don't do anything if we're not doing a third coral

            coralCount > 2 ? ( // Intake a fourth coral if we scored the third
                // Move from the reef to the human player station
                new FollowPathCommand(getChoreoTrajectory(FOURTH_CORAL_INTAKE.getPathName(redOrBlue)), Suppliers.isRedAlliance, "FourthCoralIntakeChoreoPath", 0.5, false, false)
                // While moving, stow (after waiting a moment to clear the reef)
                .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow())))
                // Once we're stowed and at the human player station, intake
                .andThen(scoring.intakeUntilHasCoralCommand())
            ):Commands.none(), // If we didn't score a third coral, don't intake another one

            coralCount > 3 ? ( // If we're scoring a fourth coral
                ( // Move from our start position to the reef, cutting the path off in the middle to activate DTT
                    new FollowPathCommand(
                        getChoreoTrajectory(FOURTH_CORAL_SCORE.getPathName(redOrBlue)),//, 0),
                        Suppliers.isRedAlliance,
                        "FourthCoralScoreChoreoPath",
                        0.5,
                        false,
                        true
                    )
                    // .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(FOURTH_CORAL_SCORE.getLeftOrRight(redOrBlue));}))
                    // .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
                )
                // While running path and DTT, raise the scoring mech to L4 position
                .alongWith(Commands.none().andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4())))
                // Once both the path and scoring mechanism are finished, score the first coral
                .andThen(scoring.outtakeCoralCommand().withTimeout(0.125))
            ):Commands.none(), // Do nothing if we aren't scoring a fourth coral

            coralCount > 3 ? ( // Only move back from the fourth coral's scoring position if we actually went there and scored
                new FollowPathCommand(getChoreoTrajectory(FOURTH_CORAL_BACK_UP.getPathName(redOrBlue)), Suppliers.isRedAlliance,"FourthCoralBackUpChoreoPath")
                .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow())))
            ):Commands.none()
        );
    }

}

//.alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow()))
