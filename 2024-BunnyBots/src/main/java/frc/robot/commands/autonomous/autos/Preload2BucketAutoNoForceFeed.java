package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Suppliers;
import frc.robot.Constants.INTAKE;
import frc.robot.commands.NewtonCommands;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.*;
import frc.robot.commands.proxies.TimingSimulatedCommand;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.Positions;

public class Preload2BucketAutoNoForceFeed extends AutoCommand {
    
    public Preload2BucketAutoNoForceFeed() {
        super(
            new TimingSimulatedCommand(NewtonCommands.setPivotPositionCommand(Pivot.Positions.SCORE_GRID)),
            NewtonCommands.runIntakeCommand(INTAKE.TOP_MOTOR_SCORE_SPEED, INTAKE.BOTTOM_MOTOR_SCORE_SPEED).withTimeout(.5),
            new FollowPathCommand(getChoreoTrajectory("Bucket1IntakeNoForceFeed"), Suppliers.robotRunningOnRed).alongWith(
                new WaitCommand(1).andThen(
                    new TimingSimulatedCommand(NewtonCommands.setPivotPositionCommand(Positions.GROUND)),
                    NewtonCommands.runIntakeCommand(INTAKE.TOP_MOTOR_INTAKE_SPEED, INTAKE.BOTTOM_MOTOR_INTAKE_SPEED)
                )
            ),
            new WaitCommand(1),
            NewtonCommands.setPivotPositionCommand(Positions.REST).withTimeout(1),
            new FollowPathCommand(getChoreoTrajectory("Bucket1IntakeReturnNoForceFeed"), Suppliers.robotRunningOnRed).deadlineWith(
                NewtonCommands.runIntakeCommand(INTAKE.TOP_MOTOR_DEFAULT_SPEED, INTAKE.BOTTOM_MOTOR_DEFAULT_SPEED),
                new TimingSimulatedCommand(NewtonCommands.setPivotPositionCommand(Positions.REST).withTimeout(1).andThen( 
                    NewtonCommands.setPivotPositionCommand(Pivot.Positions.SCORE_GRID)
                ))
            ),
            NewtonCommands.runIntakeCommand(INTAKE.TOP_MOTOR_OUTTAKE_SPEED, INTAKE.BOTTOM_MOTOR_OUTTAKE_SPEED).withTimeout(1.5)
        );
        setStartStateFromChoreoTrajectory("Bucket1Intake");
    }

}
