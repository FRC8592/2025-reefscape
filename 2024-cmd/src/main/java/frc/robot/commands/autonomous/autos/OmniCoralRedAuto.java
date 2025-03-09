package frc.robot.commands.autonomous.autos;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.ScoreCoral.LeftOrRight;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;
import static frc.robot.commands.autonomous.autos.OmniCoralRedAuto.Positions.*;

public class OmniCoralRedAuto extends AutoCommand{
    protected enum Positions{
        FIRST_CORAL_SCORE("RightToCLeft", "LeftToERight"),
        SECOND_CORAL_INTAKE("CLeftToHPRight", "ERightToHPLeft"),
        SECOND_CORAL_SCORE("HPRightToBRight", "HPLeftToFLeft"),
        THIRD_CORAL_INTAKE("BRightToHPRight", "FLeftToHPLeft"),
        THIRD_CORAL_SCORE("HPRightToBLeft", "HPLeftToFRight"),
        FOURTH_CORAL_INTAKE("BLeftToHPRight", "FRightToHPLeft"),
        FOURTH_CORAL_SCORE("HPRightToARight", "HPLeftToALeft"),
        FOURTH_CORAL_BACK_UP("ARightBackUp", "ALeftBackUp")
        ;
        String red, blue;
        private Positions(String redName, String blueName){
            this.red = redName;
            this.blue = blueName;
        }
        public String getPathName(RedOrBlue redOrBlue){
            if(redOrBlue == RedOrBlue.RED){
                return red;
            }
            else{
                return blue;
            }
        }
    }
    public enum RedOrBlue{
        RED,
        BLUE,;
    }
    public OmniCoralRedAuto(int coralCount, RedOrBlue redOrBlue){
        super(
            // NOTE: DTT = drive-to-tag
            coralCount > 0 ? ( // Score the first coral if coralcount > 0
                ( // Move from our start position to the reef, cutting the path off in the middle to activate DTT
                    new FollowPathCommand(
                        getChoreoTrajectory(FIRST_CORAL_SCORE.getPathName(redOrBlue), 0),
                        Suppliers.robotRunningOnRed
                    ).withTimeout(3.25)
                    .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(Suppliers.robotRunningOnRed.getAsBoolean()?LeftOrRight.Right:LeftOrRight.Left);}))
                    .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
                )
                // While running path and DTT, raise the scoring mech to L4 position
                .alongWith(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4()))
                // Once both the path and scoring mechanism are finished, score the first coral
                .andThen(new WaitCommand(0.25), scoring.outtakeCoralCommand().withTimeout(0.75))
            ):Commands.none(), // Commands.none() if coralcount < 1

            coralCount > 0 ? ( // Also intake the next coral if coralcount > 0
                // Move from the reef to the human player station
                new FollowPathCommand(getChoreoTrajectory(SECOND_CORAL_INTAKE.getPathName(redOrBlue)), Suppliers.robotRunningOnRed)
                // While moving, stow (after waiting a moment to clear the reef)
                .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow())))
                // Once we're stowed and at the human player station, intake
                .andThen(scoring.intakeUntilHasCoralCommand())
            ):Commands.none(), // Do not intake if coralcount < 1

            coralCount > 1 ? ( // Score the second coral if coralcount > 1
                ( // Move from our start position to the reef, cutting the path off in the middle to activate DTT
                    new FollowPathCommand(
                        getChoreoTrajectory(SECOND_CORAL_SCORE.getPathName(redOrBlue), 0),
                        Suppliers.robotRunningOnRed
                    ).withTimeout(3.25)
                    .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(Suppliers.robotRunningOnRed.getAsBoolean()?LeftOrRight.Left:LeftOrRight.Right);}))
                    .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
                )
                // While running path and DTT, raise the scoring mech to L4 position
                .alongWith(new WaitCommand(0.5).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4())))
                // Once both the path and scoring mechanism are finished, score the first coral
                .andThen(new WaitCommand(0.25), scoring.outtakeCoralCommand().withTimeout(0.75))
            ):Commands.none(), // Don't do anything if coralcount < 2

            coralCount > 1 ? ( // Intake if we scored our second coral
                // Move from the reef to the human player station
                new FollowPathCommand(getChoreoTrajectory(THIRD_CORAL_INTAKE.getPathName(redOrBlue)), Suppliers.robotRunningOnRed)
                // While moving, stow (after waiting a moment to clear the reef)
                .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow())))
                // Once we're stowed and at the human player station, intake
                .andThen(scoring.intakeUntilHasCoralCommand())
            ):Commands.none(), // Don't do anything if we're not doing a second coral

            coralCount > 2 ? ( // Score the third coral if coralcount > 2
                ( // Move from our start position to the reef, cutting the path off in the middle to activate DTT
                    new FollowPathCommand(
                        getChoreoTrajectory(THIRD_CORAL_SCORE.getPathName(redOrBlue), 0),
                        Suppliers.robotRunningOnRed
                    )
                    .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(Suppliers.robotRunningOnRed.getAsBoolean()?LeftOrRight.Right:LeftOrRight.Left);}))
                    .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
                )
                // While running path and DTT, raise the scoring mech to L4 position
                .alongWith(new WaitCommand(0.5).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4())))
                // Once both the path and scoring mechanism are finished, score the first coral
                .andThen(new WaitCommand(0.25), scoring.outtakeCoralCommand().withTimeout(0.75))
            ):Commands.none(), // Don't do anything if we're not doing a third coral

            coralCount > 2 ? ( // Intake a fourth coral if we scored the third
                // Move from the reef to the human player station
                new FollowPathCommand(getChoreoTrajectory(FOURTH_CORAL_INTAKE.getPathName(redOrBlue)), Suppliers.robotRunningOnRed)
                // While moving, stow (after waiting a moment to clear the reef)
                .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow())))
                // Once we're stowed and at the human player station, intake
                .andThen(scoring.intakeUntilHasCoralCommand())
            ):Commands.none(), // If we didn't score a third coral, don't intake another one

            coralCount > 3 ? ( // If we're scoring a fourth coral
                ( // Move from our start position to the reef, cutting the path off in the middle to activate DTT
                    new FollowPathCommand(
                        getChoreoTrajectory(FOURTH_CORAL_SCORE.getPathName(redOrBlue), 0),
                        Suppliers.robotRunningOnRed
                    )
                    .andThen(Commands.runOnce(() -> {scoreCoral.setPosition(Suppliers.robotRunningOnRed.getAsBoolean()?LeftOrRight.Left:LeftOrRight.Right);}))
                    .andThen(Commands.defer(() -> scoreCoral.driveToClosestReefTag(), Set.of(swerve)))
                )
                // While running path and DTT, raise the scoring mech to L4 position
                .alongWith(new WaitCommand(0.5).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getL4())))
                // Once both the path and scoring mechanism are finished, score the first coral
                .andThen(new WaitCommand(0.25), scoring.outtakeCoralCommand().withTimeout(0.75))
            ):Commands.none(), // Do nothing if we aren't scoring a fourth coral

            coralCount > 3 ? ( // Only move back from the fourth coral's scoring position if we actually went there and scored
                new FollowPathCommand(getChoreoTrajectory(FOURTH_CORAL_BACK_UP.getPathName(redOrBlue)), Suppliers.robotRunningOnRed)
                .alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow())))
            ):Commands.none()
        );
    }

}

//.alongWith(new WaitCommand(1).andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStow()))
