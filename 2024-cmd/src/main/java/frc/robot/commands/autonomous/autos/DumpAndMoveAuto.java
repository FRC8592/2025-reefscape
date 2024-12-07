package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Suppliers;
import frc.robot.Constants.INTAKE;
import frc.robot.commands.NewtonCommands;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.*;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.Positions;

public class DumpAndMoveAuto extends AutoCommand {
    
    public DumpAndMoveAuto() {
        super(
            NewtonCommands.setPivotPositionCommand(Pivot.Positions.SCORE_GRID),
            NewtonCommands.runIntakeCommand(INTAKE.TOP_MOTOR_SCORE_SPEED, INTAKE.BOTTOM_MOTOR_SCORE_SPEED).withTimeout(.5),
            new FollowPathCommand(getChoreoTrajectory("Bucket1Intake"), Suppliers.robotRunningOnRed)
        );
    }

}
