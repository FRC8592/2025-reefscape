package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Suppliers;
import frc.robot.Constants.INTAKE;
import frc.robot.commands.NewtonCommands;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.*;
import frc.robot.subsystems.Pivot;

public class PreloadBucket2Auto extends AutoCommand {
    
    public PreloadBucket2Auto() {
        super(
            NewtonCommands.setPivotPositionCommand(Pivot.Positions.SCORE_GRID).andThen(),
            NewtonCommands.runIntakeCommand(INTAKE.TOP_MOTOR_SCORE_SPEED, INTAKE.BOTTOM_MOTOR_SCORE_SPEED),
            new FollowPathCommand(getChoreoTrajectory("PreloadBucket2Path"), Suppliers.robotRunningOnRed)
            .andThen(NewtonCommands.runIntakeCommand(INTAKE.TOP_MOTOR_INTAKE_SPEED, INTAKE.BOTTOM_MOTOR_INTAKE_SPEED))
            .andThen(
                new FollowPathCommand(getChoreoTrajectory("PreloadBucket2Path2"), Suppliers.robotRunningOnRed)
            )
        );
        setStartStateFromChoreoTrajectory("PreloadBucket2Path");
    }

}
